mod util;
mod simulation;

use Result::{Ok};
use ws::{listen, Message};

fn main() {
    listen("127.0.0.1:8080", |out| {
        move |msg: Message| {
            if msg.as_text().unwrap() == "step" {
                out.send("{ \"vehicles\": [{ \"id\": 0, \"link\": 0, \"pos\": 50, \"lat\": 0, \"vlat\": 0, \"comm\": false }] }");
            }
            Ok(())
        }
    }).unwrap();
}