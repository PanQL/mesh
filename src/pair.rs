use nalgebra::base::Matrix4;
use nalgebra::base::Vector3;
use nalgebra::base::Vector4;
use std::cmp::Ordering;
use ordered_float::OrderedFloat;

pub struct PairInfo {
    pub id : usize,
    pub value : OrderedFloat<f64>,
}

impl Ord for PairInfo {
    fn cmp(&self, other : &Self) -> Ordering {
        other.value.cmp(&self.value)
    }
}

impl PartialOrd for PairInfo {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for PairInfo {
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

impl Eq for PairInfo {}

pub struct Pair {
    valid : bool,   // 是否是有效边
    access : bool,  // 是否有某一个顶点已经发生改变
    first : usize,  // 第一个顶点
    second : usize, // 第二个顶点
    shrink_value : f64, // 二次误差损失代价
    pub q_matrix : Matrix4<f64>,    // Q矩阵
    best_point : Vector3<f64>,  // 最优塌缩点
}

impl Pair {
    pub fn new(first : usize, second : usize) -> Self {
        Pair {
            valid : true,
            access : false,
            first,
            second,
            shrink_value : 0.0,
            q_matrix : Matrix4::zeros(),
            best_point : Vector3::zeros(),
        }
    }

    pub fn get_vertex(&self) -> (usize, usize) {
        (self.first, self.second)
    }

    pub fn get_value(&self) -> f64 {
        self.shrink_value
    }

    pub fn get_best_point(&self) -> Vector3<f64> {
        self.best_point
    }

    pub fn cal_shrink_value(&mut self) {
        let vec = Vector4::new(self.best_point.x, self.best_point.y, self.best_point.z, 1.0);
        self.shrink_value = vec.dot(&(self.q_matrix * vec));
    }

    pub fn cal_best_point(&mut self, node1 : &Vector3<f64>, node2 : &Vector3<f64>) {
        // 获取Q的线性优化矩阵
        let m = Matrix4::from_fn(|r, c| {
            let mut ret : f64 = 0.0;
            if r == 3 {
                if c == 3 {
                    ret = 1.0;
                }
            } else {
                ret = self.q_matrix[r.min(c) * 4 + r.max(c)];
            }
            ret
        });
        if let Ok(q_inverse) = m.pseudo_inverse(1e-30) {
            let col = q_inverse.column(3);
            self.best_point.x = col.x;
            self.best_point.y = col.y;
            self.best_point.z = col.z;
        } else {
            self.best_point = ( node1 + node2 ) / 2.0f64;
        }
    }

    pub fn destroy(&mut self) {
        self.valid = false;
    }

    pub fn set_vertex(&mut self, pos : &usize, code : usize) -> bool {
        self.access = true;
        if *pos == 1 {
            self.first = code;
            if self.second != code {
                return true;
            }
        } else if *pos == 2 {
            self.second = code;
            if self.first != code {
                return true;
            }
        } else {
            assert!(false);
        }
        false
    }

    pub fn access(&self) -> bool {
        self.access
    }

    pub fn valid(&self) -> bool {
        self.valid
    }

    pub fn clear_access(&mut self) {
        self.access = false;
    }
}
