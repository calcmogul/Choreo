fn main() {
    std::env::set_var("TAURI_CONFIG", "{ \"bundle\": { \"externalBin\": null } }");
    tauri_build::build();
    std::env::set_var("TAURI_CONFIG", "{}");
}
