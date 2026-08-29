//! RMW-native `geometry_msgs` (layout matches `rosidl_generator_c` on Jazzy).
//!
//! Isolated `unsafe` for the C type-support and init/fini sequence functions.
//! The rest of `flight-ros2` stays safe.

#![allow(unsafe_code)]

use std::borrow::Cow;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[link(name = "geometry_msgs__rosidl_typesupport_c")]
unsafe extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Vector3(
    ) -> *const std::ffi::c_void;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Twist(
    ) -> *const std::ffi::c_void;
}

#[link(name = "geometry_msgs__rosidl_generator_c")]
unsafe extern "C" {
    fn geometry_msgs__msg__Vector3__init(msg: *mut Vector3) -> bool;
    fn geometry_msgs__msg__Vector3__Sequence__init(
        seq: *mut rosidl_runtime_rs::Sequence<Vector3>,
        size: usize,
    ) -> bool;
    fn geometry_msgs__msg__Vector3__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Vector3>);
    fn geometry_msgs__msg__Vector3__Sequence__copy(
        in_seq: &rosidl_runtime_rs::Sequence<Vector3>,
        out_seq: *mut rosidl_runtime_rs::Sequence<Vector3>,
    ) -> bool;
    fn geometry_msgs__msg__Twist__init(msg: *mut Twist) -> bool;
    fn geometry_msgs__msg__Twist__Sequence__init(
        seq: *mut rosidl_runtime_rs::Sequence<Twist>,
        size: usize,
    ) -> bool;
    fn geometry_msgs__msg__Twist__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Twist>);
    fn geometry_msgs__msg__Twist__Sequence__copy(
        in_seq: &rosidl_runtime_rs::Sequence<Twist>,
        out_seq: *mut rosidl_runtime_rs::Sequence<Twist>,
    ) -> bool;
}

impl Default for Vector3 {
    fn default() -> Self {
        // SAFETY: zeroed POD is a valid Vector3; init only writes the three doubles.
        unsafe {
            let mut msg = std::mem::zeroed();
            if !geometry_msgs__msg__Vector3__init(&mut msg) {
                panic!("geometry_msgs__msg__Vector3__init failed");
            }
            msg
        }
    }
}

impl Default for Twist {
    fn default() -> Self {
        // SAFETY: zeroed POD is a valid Twist; init writes nested Vector3 fields.
        unsafe {
            let mut msg = std::mem::zeroed();
            if !geometry_msgs__msg__Twist__init(&mut msg) {
                panic!("geometry_msgs__msg__Twist__init failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for Vector3 {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        // SAFETY: `seq` is a valid Sequence allocated by rosidl.
        unsafe { geometry_msgs__msg__Vector3__Sequence__init(seq as *mut _, size) }
    }
    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        // SAFETY: `seq` was initialized by sequence_init or the C generator.
        unsafe { geometry_msgs__msg__Vector3__Sequence__fini(seq as *mut _) }
    }
    fn sequence_copy(
        in_seq: &rosidl_runtime_rs::Sequence<Self>,
        out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
    ) -> bool {
        // SAFETY: both sequences are live rosidl sequences of Vector3.
        unsafe { geometry_msgs__msg__Vector3__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for Twist {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        // SAFETY: `seq` is a valid Sequence allocated by rosidl.
        unsafe { geometry_msgs__msg__Twist__Sequence__init(seq as *mut _, size) }
    }
    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        // SAFETY: `seq` was initialized by sequence_init or the C generator.
        unsafe { geometry_msgs__msg__Twist__Sequence__fini(seq as *mut _) }
    }
    fn sequence_copy(
        in_seq: &rosidl_runtime_rs::Sequence<Self>,
        out_seq: &mut rosidl_runtime_rs::Sequence<Self>,
    ) -> bool {
        // SAFETY: both sequences are live rosidl sequences of Twist.
        unsafe { geometry_msgs__msg__Twist__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for Vector3 {
    type RmwMsg = Self;
    fn into_rmw_message(msg_cow: Cow<'_, Self>) -> Cow<'_, Self::RmwMsg> {
        msg_cow
    }
    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for Vector3 {
    const TYPE_NAME: &'static str = "geometry_msgs/msg/Vector3";
    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: typesupport handle is a process-lifetime pointer from the C library.
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Vector3()
        }
    }
}

impl rosidl_runtime_rs::Message for Twist {
    type RmwMsg = Self;
    fn into_rmw_message(msg_cow: Cow<'_, Self>) -> Cow<'_, Self::RmwMsg> {
        msg_cow
    }
    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for Twist {
    const TYPE_NAME: &'static str = "geometry_msgs/msg/Twist";
    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: typesupport handle is a process-lifetime pointer from the C library.
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Twist()
        }
    }
}

impl Twist {
    pub fn from_ned_velocity(v: flight_core::vector::Velocity<flight_core::frames::Ned>) -> Self {
        let [x, y, z] = crate::ned_velocity_to_ros_twist_linear(v);
        Self {
            linear: Vector3 { x, y, z },
            angular: Vector3::default(),
        }
    }
}
