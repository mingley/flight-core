fn main() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=ROS_DISTRO");
    if std::env::var("CARGO_FEATURE_RCLRS").is_err() {
        return;
    }
    let prefix = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    for p in prefix.split(':').filter(|s| !s.is_empty()) {
        let lib = format!("{p}/lib");
        println!("cargo:rustc-link-search=native={lib}");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{lib}");
    }
    println!("cargo:rustc-link-lib=dylib=geometry_msgs__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=dylib=geometry_msgs__rosidl_generator_c");
}
