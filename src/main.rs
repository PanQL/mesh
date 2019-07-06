mod pair;
mod vertex;
mod face;
mod mesh;

extern crate nalgebra;
use std::path::Path;
use std::env;
use std::vec::Vec;
use mesh::Mesh;
use std::str::FromStr;

fn main() {
    let args : Vec<String> = env::args().collect();
    println!("{:?}", args);
    assert!( args.len() > 3 );
    let mut mesh = Mesh::default();
    mesh.load(&args[1]);
    mesh.cal_q_v();
    mesh.init_pairs();
    mesh.run(f32::from_str(&args[3]).unwrap());
    mesh.save(&Path::new(&args[2]));
}
