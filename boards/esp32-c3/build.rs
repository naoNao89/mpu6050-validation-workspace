fn main() {
    // Required by embedded-test to preserve test metadata in the target ELF.
    println!("cargo::rustc-link-arg-tests=-Tembedded-test.x");

    // Register embedded-test's rust-analyzer cfg to avoid unexpected_cfgs warnings.
    println!("cargo::rustc-check-cfg=cfg(rust_analyzer)");
}
