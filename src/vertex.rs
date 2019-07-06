use nalgebra::Vector3;
use nalgebra::base::Matrix4;
use std::vec::Vec;

pub struct Vertex{
    valid : bool,
    pub coord : Vector3<f64>,
    pub q_v : Matrix4<f64>,
    pub faces : Vec<(usize, usize)>,    // 面片编号，在面片中的位置
    pub pairs : Vec<(usize, usize)>,    // pair编号，在pair中的位置
}

impl Vertex{
    pub fn new(x : f64, y : f64, z : f64) -> Self {
        Vertex {
            valid : true,
            coord : Vector3::new(x, y, z),
            q_v : Matrix4::zeros(),
            faces : Vec::new(),
            pairs : Vec::new(),
        }
    }

    pub fn add_face(&mut self, index : usize, position : usize) {
        self.faces.push((index, position));
    }

    pub fn add_pair(&mut self, index : usize, position : usize) {
        self.pairs.push((index, position));
    }

    pub fn destroy(&mut self) {
        self.valid = false;
    }

    pub fn renew_state(&mut self) {
        self.valid = true;
        self.faces.clear();
        self.pairs.clear();
    }

    pub fn valid(&self) -> bool {
        self.valid
    }

    pub fn distance(this : &Vector3<f64>, other : &Vector3<f64>) -> f64 {
        ((this.x - other.x).powi(2) 
         + (this.y - other.y).powi(2) 
         + (this.z - other.z).powi(2)).sqrt()
    }

    pub fn get_string(&self) -> String {
        format!("v {} {} {} \n", self.coord[0], self.coord[1], self.coord[2])
    }
}
