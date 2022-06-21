// Configuration
pub enum Config {
    SSID, PASSWORD, APIKEY
}

pub fn get_config(value: Config) -> &'static str {
    match value {
        Config::SSID => (return "SSIDDUMMY"),
        Config::PASSWORD => (return "PASSWORDDUMMY"),
        Config::APIKEY => (return "APIKEYDUMMY"),
    }
}
