use nalgebra::base::Matrix4;

pub struct Face{
    valid : bool,
    pub indices : [usize;3],
    pub k_p : Matrix4<f64>,
}

impl Face {
    pub fn new(a : usize, b : usize, c : usize, k_p : Matrix4<f64>) -> Self {
        Face {
            valid : true,
            indices : [a, b, c],
            k_p,
        }
    }

    pub fn destroy(&mut self) {
        self.valid = false;
    }

    pub fn valid(&self) -> bool {
        self.valid
    }

    pub fn both_in(&self, except : &usize, idx : &usize) -> bool {
        if self.indices[(*except + 1) % 3] == *idx {
            return true;
        }
        if self.indices[(*except + 2) % 3] == *idx {
            return true;
        }
        false
    }

    pub fn get_string(&self) -> String {
        format!("f {} {} {} \n", self.indices[0], self.indices[1], self.indices[2])
    }
}

