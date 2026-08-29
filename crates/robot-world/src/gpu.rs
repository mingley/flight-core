//! Vulkan compute dispatch of the Saint-Venant sweep.
//!
//! The shader is the same Rusanov face flux as `flight_core::hydro`. GPU is a
//! **performance path**: `gpu_or_cpu_coastal_holds` checks hydro invariants,
//! not CPU/GPU bitwise identity. If this machine has no Vulkan adapter
//! (including lavapipe), callers fall back to the CPU kernel.

use crate::env::Environment;
use crate::hydro::HydroField;
use flight_core::hydro::{hydro_cfl_substeps, HydroState};
use std::sync::OnceLock;
use wgpu::util::DeviceExt;

pub fn active() -> bool {
    requested() && backend().is_some()
}

pub(crate) fn requested() -> bool {
    matches!(
        std::env::var("FLIGHT_HYDRO_GPU").ok().as_deref(),
        Some("1") | Some("true") | Some("TRUE")
    )
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Params {
    nx: u32,
    ny: u32,
    dx: f32,
    dt: f32,
    g: f32,
    along_n: u32,
    cells: u32,
    _pad: u32,
}

struct HydroGpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::ComputePipeline,
    layout: wgpu::BindGroupLayout,
}

fn backend() -> Option<&'static HydroGpu> {
    static GPU: OnceLock<Option<HydroGpu>> = OnceLock::new();
    GPU.get_or_init(HydroGpu::try_new).as_ref()
}

/// Run the hydro step on the GPU. Returns false if no adapter is available.
pub fn advance(field: &mut HydroField, dt: f32, env: &Environment) -> bool {
    let Some(gpu) = backend() else {
        return false;
    };
    if !(dt.is_finite() && dt > 0.0 && dt < 1.0) {
        return true;
    }
    let nsub = hydro_cfl_substeps(field.grid, &field.h, &field.un, &field.ue, dt);
    let dti = dt / nsub as f32;
    for _ in 0..nsub {
        if gpu.sweep(field, true, dti).is_err() {
            return false;
        }
        if gpu.sweep(field, false, dti).is_err() {
            return false;
        }
        let mut state = HydroState {
            grid: field.grid,
            h: &mut field.h,
            un: &mut field.un,
            ue: &mut field.ue,
            still: &field.still,
            scratch: &mut field.scratch,
        };
        state.relax(dti, env.wind_ned[0], env.wind_ned[1]);
    }
    true
}

impl HydroGpu {
    fn try_new() -> Option<Self> {
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(Self::try_new_inner))
            .ok()
            .flatten()
    }

    fn try_new_inner() -> Option<Self> {
        // Prefer lavapipe when present so CI / cloud VMs still run the shader.
        if std::env::var_os("VK_ICD_FILENAMES").is_none() {
            let lvp = "/usr/share/vulkan/icd.d/lvp_icd.json";
            if std::path::Path::new(lvp).exists() {
                std::env::set_var("VK_ICD_FILENAMES", lvp);
            }
        }
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::VULKAN,
            ..Default::default()
        });
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::LowPower,
            compatible_surface: None,
            force_fallback_adapter: true,
        }))?;
        let (device, queue) = pollster::block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("flight-hydro"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::downlevel_defaults(),
                memory_hints: wgpu::MemoryHints::Performance,
            },
            None,
        ))
        .ok()?;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("hydro-sweep"),
            source: wgpu::ShaderSource::Wgsl(include_str!("hydro.wgsl").into()),
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("hydro-bgl"),
            entries: &[
                storage_entry(0, wgpu::BufferBindingType::Uniform),
                storage_entry(1, wgpu::BufferBindingType::Storage { read_only: true }),
                storage_entry(2, wgpu::BufferBindingType::Storage { read_only: false }),
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("hydro-pl"),
            bind_group_layouts: &[&layout],
            push_constant_ranges: &[],
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("hydro-sweep-pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader,
            entry_point: Some("sweep"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        });
        Some(Self {
            device,
            queue,
            pipeline,
            layout,
        })
    }

    fn sweep(&self, field: &mut HydroField, along_n: bool, dt: f32) -> Result<(), ()> {
        let n = field.grid.cells();
        let params = Params {
            nx: field.grid.nx as u32,
            ny: field.grid.ny as u32,
            dx: field.grid.dx,
            dt,
            g: field.grid.g,
            along_n: u32::from(along_n),
            cells: n as u32,
            _pad: 0,
        };
        let param_buf = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("hydro-params"),
                contents: bytemuck::bytes_of(&params),
                usage: wgpu::BufferUsages::UNIFORM,
            });
        let mut packed = Vec::with_capacity(4 * n);
        packed.extend_from_slice(&field.h);
        packed.extend_from_slice(&field.un);
        packed.extend_from_slice(&field.ue);
        packed.extend_from_slice(&field.still);
        let src = self.storage(&packed, true);
        let dst = self.storage(&vec![0.0; 3 * n], false);
        let bind = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("hydro-bg"),
            layout: &self.layout,
            entries: &[bind(0, &param_buf), bind(1, &src), bind(2, &dst)],
        });
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("hydro-enc"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("hydro-pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(n.div_ceil(64) as u32, 1, 1);
        }
        let stage = self.staging(3 * n);
        encoder.copy_buffer_to_buffer(&dst, 0, &stage, 0, (3 * n * 4) as u64);
        self.queue.submit(Some(encoder.finish()));
        let mut out = vec![0.0; 3 * n];
        read_f32(&self.device, &stage, &mut out)?;
        field.h.copy_from_slice(&out[..n]);
        field.un.copy_from_slice(&out[n..2 * n]);
        field.ue.copy_from_slice(&out[2 * n..3 * n]);
        Ok(())
    }

    fn storage(&self, data: &[f32], read_only: bool) -> wgpu::Buffer {
        let usage = if read_only {
            wgpu::BufferUsages::STORAGE
        } else {
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC
        };
        self.device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: None,
                contents: bytemuck::cast_slice(data),
                usage,
            })
    }

    fn staging(&self, n: usize) -> wgpu::Buffer {
        self.device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: (n * 4) as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    }
}

fn storage_entry(binding: u32, ty: wgpu::BufferBindingType) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn bind(binding: u32, buffer: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
    wgpu::BindGroupEntry {
        binding,
        resource: buffer.as_entire_binding(),
    }
}

fn read_f32(device: &wgpu::Device, buf: &wgpu::Buffer, dst: &mut [f32]) -> Result<(), ()> {
    let slice = buf.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device.poll(wgpu::Maintain::Wait);
    rx.recv().map_err(|_| ())?.map_err(|_| ())?;
    {
        let data = slice.get_mapped_range();
        dst.copy_from_slice(bytemuck::cast_slice(&data));
    }
    buf.unmap();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::env::Environment;

    #[test]
    fn gpu_or_cpu_coastal_holds() {
        let mut field = HydroField::from_env(&Environment::coastal());
        let used_gpu = advance(&mut field, 0.02, &Environment::coastal());
        for _ in 0..30 {
            let env = Environment::coastal();
            if used_gpu {
                assert!(advance(&mut field, 0.02, &env));
            } else {
                field.step(0.02, &env);
            }
            assert!(field.invariants().all(), "{:?}", field.invariants());
        }
        if used_gpu {
            eprintln!("hydro ran on Vulkan compute");
        } else {
            eprintln!("no Vulkan adapter; CPU hydro only");
        }
    }
}
