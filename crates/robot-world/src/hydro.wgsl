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

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> src: array<f32>;
@group(0) @binding(2) var<storage, read_write> dst: array<f32>;

const H_DRY: f32 = 0.0001;

fn h_at(k: u32) -> f32 { return src[k]; }
fn un_at(k: u32) -> f32 { return src[params.cells + k]; }
fn ue_at(k: u32) -> f32 { return src[2u * params.cells + k]; }
fn still_at(k: u32) -> f32 { return src[3u * params.cells + k]; }

fn idx(i: i32, j: i32) -> u32 {
    return u32(i) * params.ny + u32(j);
}

fn in_bounds(i: i32, j: i32) -> bool {
    return i >= 0 && j >= 0 && i < i32(params.nx) && j < i32(params.ny);
}

fn wet(i: i32, j: i32) -> bool {
    if !in_bounds(i, j) {
        return false;
    }
    return still_at(idx(i, j)) > 0.0;
}

fn rusanov(hl: f32, ul: f32, hr: f32, ur: f32) -> vec2<f32> {
    let h_l = max(hl, 0.0);
    let h_r = max(hr, 0.0);
    let u_l = select(ul, 0.0, h_l < H_DRY);
    let u_r = select(ur, 0.0, h_r < H_DRY);
    let fl_m = h_l * u_l;
    let fr_m = h_r * u_r;
    let fl_q = select(0.0, h_l * u_l * u_l + 0.5 * params.g * h_l * h_l, h_l >= H_DRY);
    let fr_q = select(0.0, h_r * u_r * u_r + 0.5 * params.g * h_r * h_r, h_r >= H_DRY);
    let cl = select(0.0, sqrt(max(params.g * h_l, 0.0)), h_l >= H_DRY);
    let cr = select(0.0, sqrt(max(params.g * h_r, 0.0)), h_r >= H_DRY);
    let s = max(abs(u_l) + cl, abs(u_r) + cr);
    return vec2(
        0.5 * (fl_m + fr_m) - 0.5 * s * (h_r - h_l),
        0.5 * (fl_q + fr_q) - 0.5 * s * (h_r * u_r - h_l * u_l),
    );
}

fn axis_u(k: u32) -> f32 {
    return select(un_at(k), ue_at(k), params.along_n == 0u);
}

fn face(i: i32, j: i32, di: i32, dj: i32) -> vec2<f32> {
    let ri = i + di;
    let rj = j + dj;
    let lw = wet(i, j);
    let rw = wet(ri, rj);
    if lw && rw {
        let l = idx(i, j);
        let r = idx(ri, rj);
        return rusanov(h_at(l), axis_u(l), h_at(r), axis_u(r));
    }
    if lw {
        let l = idx(i, j);
        let u_l = axis_u(l);
        return rusanov(h_at(l), u_l, h_at(l), -u_l);
    }
    if rw {
        let r = idx(ri, rj);
        let u_r = axis_u(r);
        return rusanov(h_at(r), -u_r, h_at(r), u_r);
    }
    return vec2(0.0, 0.0);
}

@compute @workgroup_size(64)
fn sweep(@builtin(global_invocation_id) gid: vec3<u32>) {
    let k = gid.x;
    if k >= params.cells {
        return;
    }
    let i = i32(k / params.ny);
    let j = i32(k % params.ny);
    if still_at(k) <= 0.0 {
        dst[k] = 0.0;
        dst[params.cells + k] = 0.0;
        dst[2u * params.cells + k] = 0.0;
        return;
    }
    var di: i32 = 1;
    var dj: i32 = 0;
    if params.along_n == 0u {
        di = 0;
        dj = 1;
    }
    let fp = face(i, j, di, dj);
    let fm = face(i - di, j - dj, di, dj);
    let inv = params.dt / max(params.dx, 1e-6);
    let h0 = max(h_at(k), 0.0);
    var h1 = max(h0 - inv * (fp.x - fm.x), 0.0);
    let u0 = axis_u(k);
    var q1 = h0 * u0 - inv * (fp.y - fm.y);
    var u1 = select(q1 / h1, 0.0, h1 < H_DRY);
    dst[k] = h1;
    if params.along_n == 1u {
        dst[params.cells + k] = u1;
        dst[2u * params.cells + k] = ue_at(k);
    } else {
        dst[params.cells + k] = un_at(k);
        dst[2u * params.cells + k] = u1;
    }
}
