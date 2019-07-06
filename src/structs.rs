use super::*;

pub struct Face{
    indices : [u32;3],
}

impl Face {
    pub fn new(a : u32, b : u32, c : u32) -> Self {
        Face {
            indices : [a, b, c],
        }
    }
}

