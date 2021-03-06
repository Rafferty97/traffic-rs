mod util;
mod simulation;

use util::{LinearFunc, CubicFunc};
use Result::{Ok};
use std::io::{self, Write};
use byteorder::{NetworkEndian as NE, WriteBytesExt};
use ws::{listen, Message};

struct Client {
    out: ws::Sender,
    sim: simulation::Simulation
}

impl Client {
    fn step<W: Write>(&mut self, w: &mut W) -> io::Result<()> {
        self.sim.step();
        // Code for a frame message
        w.write_u32::<NE>(1)?;
        // The frame index
        w.write_u32::<NE>(self.sim.get_step() as u32)?;
        // The vehicle positions
        for veh in self.sim.get_vehicle_states() {
            w.write_u32::<NE>(veh.user_id as u32)?;
            w.write_u32::<NE>(veh.link as u32)?;
            w.write_f32::<NE>(veh.pos)?;
            w.write_f32::<NE>(veh.vel)?;
            w.write_f32::<NE>(veh.lat)?;
            w.write_f32::<NE>(veh.dlat)?;
        }
        w.write_u32::<NE>(!0)?;
        // Everything succeeded
        Ok(())
    }
}

impl ws::Handler for Client {
    fn on_message(&mut self, msg: Message) -> ws::Result<()> {
        let text = msg.as_text().unwrap();
        let mut parts = text.split(" ");
        let msg_type = parts.next().unwrap();

        if msg_type == "start" {
            let delta: f32 = parts.next().unwrap().parse().unwrap();
            self.sim = simulation::Simulation::new(delta);
        }

        if msg_type == "veh" {
            let id: usize = parts.next().unwrap().parse().unwrap();
            let src_link: usize = parts.next().unwrap().parse().unwrap();
            let dst_link: usize = parts.next().unwrap().parse().unwrap();
            let lane: u8 = parts.next().unwrap().parse().unwrap();
            let pos: f32 = parts.next().unwrap().parse().unwrap();
            let veh_id = self.sim.add_vehicle(id);
            self.sim.set_vehicle_pos(veh_id, src_link, lane, pos);
            self.sim.set_vehicle_dest(veh_id, dst_link);
            // todo: route to dst_link
        }

        if msg_type == "link" {
            let link: usize = parts.next().unwrap().parse().unwrap();
            let length: f32 = parts.next().unwrap().parse().unwrap();
            let lanes = parts.next().unwrap();
            let speed_limit: f32 = parts.next().unwrap().parse().unwrap();
            self.sim.add_link(link, length, speed_limit);
            for lane in lanes.split(";") {
                // todo: curvature
                // todo: more complex lateral curves
                let mut lats = lane.split(",").map(|x| x.parse::<f32>().unwrap());
                let dist_func = LinearFunc::from_points(&[(0.0, 0.0), (length, length)]);
                let lat_func = CubicFunc::from_points(&[(0.0, lats.next().unwrap()), (length, lats.next().unwrap())]);
                self.sim.add_lane(link, dist_func, lat_func);
            }
        }

        if msg_type == "conn" {
            let src_link: usize = parts.next().unwrap().parse().unwrap();
            let dst_link: usize = parts.next().unwrap().parse().unwrap();
            let lanes = parts.next().unwrap();
            let offset: f32 = parts.next().unwrap().parse().unwrap();
            self.sim.add_connection(src_link, dst_link, lanes, offset);
        }

        if msg_type == "stop" {
            let id: usize = parts.next().unwrap().parse().unwrap();
            let link: usize = parts.next().unwrap().parse().unwrap();
            let lane: u8 = parts.next().unwrap().parse().unwrap();
            let pos: f32 = parts.next().unwrap().parse().unwrap();
            let length: f32 = parts.next().unwrap().parse().unwrap();
            let kind: simulation::StopLineType = parts.next().unwrap().parse().unwrap();
            simulation::StopLineBuilder::new(id, link, lane, pos)
                .with_length(length)
                .of_type(kind)
                .add_to_simulation(&mut self.sim);
        }

        if msg_type == "conflict" {
            let stop1: usize = parts.next().unwrap().parse().unwrap();
            let stop2: usize = parts.next().unwrap().parse().unwrap();
            let priority: i8 = parts.next().unwrap().parse().unwrap();
            let _min_pos: f32 = parts.next().unwrap().parse().unwrap();
            let max_pos: f32 = parts.next().unwrap().parse().unwrap();
            let priority = priority.cmp(&0);
            self.sim.add_conflict(stop1, stop2, priority, max_pos);
        }

        if msg_type == "step" {
            let num_steps: usize = parts.next().unwrap().parse().unwrap();
            let mut buffer = vec![];
            for _i in 0..num_steps {
                self.step(&mut buffer).unwrap();
            }
            self.out.send(buffer).unwrap();
        }
        Ok(())
    }
}

fn main() {
    listen("0.0.0.0:8080", |out| {
        let sim = simulation::Simulation::new(1f32 / 10f32);
        Client {
            out,
            sim
        }
    }).unwrap();
}