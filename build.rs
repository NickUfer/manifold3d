use std::env;

fn main() {
    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap();

    if target_arch == "wasm32" && target_os == "emscripten" {
        println!("cargo:rustc-link-arg=--no-entry");
    }
}
