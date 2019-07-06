use super::vertex::Vertex;
use super::face::Face;
use super::pair::{ Pair, PairInfo };
use std::collections::BinaryHeap;
use nalgebra::base::Matrix4;
use std::path::Path;
use ordered_float::OrderedFloat;
use std::io::LineWriter;
use std::fs::File;
use std::io::prelude::*;
use std::io::BufReader;
use std::io::BufRead;
use std::str::FromStr;
use std::collections::HashSet;
use std::collections::VecDeque;
use std::sync::Arc;
use std::cell::RefCell;


#[derive(Default)]
pub struct Mesh {
    vertices :  Vec<Arc<RefCell<Vertex>>>,  // 顶点实体
    faces : Vec<Face>,
    pairs : Vec<Pair>,
    heap : BinaryHeap<PairInfo>,
    pre_pairs : VecDeque<HashSet<usize>>,   // 用于建立pair的数对
    trash : Vec<usize>, // 可供使用的顶点实体集合
    total : u32,
}

impl Mesh{
    pub fn load(&mut self, file_path : &str) {
        let f = File::open(file_path).unwrap();
        let file = BufReader::new(&f);
        let mut face_index = 0;

        for (_, line) in file.lines().enumerate() {
            let l = line.unwrap();
            let args : Vec<&str> = l.split(' ').collect();
            match args[0] {
                "v" => {
                    assert!( args.len() == 4);
                    self.vertices.push(Arc::new(RefCell::new(Vertex::new(
                            f64::from_str(args[1]).unwrap(),
                            f64::from_str(args[2]).unwrap(),
                            f64::from_str(args[3]).unwrap()
                            ))));
                    self.pre_pairs.push_back(HashSet::new());
                }
                "f" => {
                    // 取得三个顶点的编号
                    let idx1 = usize::from_str(args[1]).unwrap() - 1;
                    let idx2 = usize::from_str(args[2]).unwrap() - 1;
                    let idx3 = usize::from_str(args[3]).unwrap() - 1;

                    if idx1 < idx2 {
                        self.pre_pairs[idx1].insert(idx2);
                    } else {
                        assert!( idx1 != idx2);
                        self.pre_pairs[idx2].insert(idx1);
                    }
                    if idx1 < idx3 {
                        self.pre_pairs[idx1].insert(idx3);
                    } else {
                        assert!( idx1 != idx3);
                        self.pre_pairs[idx3].insert(idx1);
                    }
                    if idx2 < idx3 {
                        self.pre_pairs[idx2].insert(idx3);
                    } else {
                        assert!( idx2 != idx3);
                        self.pre_pairs[idx3].insert(idx2);
                    }

                    let mut vertex1 = self.vertices[idx1].borrow_mut();
                    let mut vertex2 = self.vertices[idx2].borrow_mut();
                    let mut vertex3 = self.vertices[idx3].borrow_mut();

                    // 为顶点和三角形面片建立连接
                    vertex1.add_face(face_index, 0);
                    vertex2.add_face(face_index, 1);
                    vertex3.add_face(face_index, 2);
                    face_index += 1;

                    // 计算三角形面片的Kp
                    let vec21 = vertex2.coord - vertex1.coord;
                    let vec31 = vertex3.coord - vertex1.coord;
                    let n = vec21.cross(&vec31).normalize(); // 法向量N
                    let d = n.dot(&vertex1.coord) * -1.0;
                    let k_p = Matrix4::from_fn(|r, c| {
                        let src1 = if r > 2 { d } else { n[r] };
                        let src2 = if c > 2 { d } else { n[c] };
                        src1 * src2
                    });
                    self.faces.push(Face::new(idx1, idx2, idx3, k_p));
                }
                _ => {
                    println!("other");
                }
            }
        }
        // 多出一个空闲顶点用于中转
        self.vertices.push(Arc::new(RefCell::new(Vertex::new(0.0f64, 0.0f64, 0.0f64))));

        println!("init vertices {}, faces {}", self.vertices.len(), self.faces.len());
    }

    pub fn cal_q_v(&mut self) {
        // 计算顶点的Qv矩阵
        for v in self.vertices.iter() {
            let mut vertex = v.borrow_mut();
            let mut q_v = Matrix4::zeros();
            for (idx, _) in vertex.faces.iter() {
                q_v += self.faces[*idx].k_p;
            }
            vertex.q_v = q_v;
        }
    }

    fn add_pair(&mut self, id1 : usize, id2 : usize) {
        assert!( id1 < id2);
        // 创建一个pair
        let mut vertex1 = self.vertices[id1].borrow_mut();
        let mut vertex2 = self.vertices[id2].borrow_mut();

        // TODO 阈值应当运行时指定
        let dist = Vertex::distance(&vertex1.coord, &vertex2.coord);
        if dist > 10.0 {
            return;
        }

        let mut pair = Pair::new(id1, id2);
        pair.q_matrix = vertex1.q_v + vertex2.q_v;
        pair.cal_best_point(&vertex1.coord, &vertex2.coord);
        pair.cal_shrink_value();

        let idx = self.pairs.len();
        let value = pair.get_value();
        self.pairs.push(pair);

        // 更新顶点信息，pairinfo信息
        let pair_info = PairInfo { id : idx, value : OrderedFloat::from(value) };
        self.heap.push(pair_info);
        vertex1.add_pair(idx.clone(), 1);
        vertex2.add_pair(idx.clone(), 2);
    }

    pub fn init_pairs(&mut self) {
        // TODO 遍历所有面片，建立pair的列表以及pairinfo的堆
        let len = self.pre_pairs.len();
        for v_idx in 0..len {
            let v_set = self.pre_pairs.pop_front().unwrap();
            for idx in v_set.iter() {
                assert!( v_idx < *idx );
                self.add_pair( v_idx, idx.clone());
            }
        }
        println!("number of pairs : {}", self.pairs.len());
    }

    pub fn run(&mut self, scale : f32) {
        // 化简过程
        self.total = self.faces.len() as u32;
        let target = ( self.total as f32 * scale ) as u32;
        println!("total is {}, target is {}", self.total, target);
        while self.total > target {
            if let Some(vectim) = self.heap.pop() {
                self.del_pair(vectim.id);
            } else {    // empty
                println!("to break!!!");
                break;
            }
            loop {
                // 检查堆头的pair是否为有效的pair,或者是否需要更新
                let mut flag = HeadType::NORMAL;
                if let Some(head) = self.heap.peek() {
                    let temp_pair = self.pairs.get(head.id).unwrap();
                    if !temp_pair.valid() {
                        flag = HeadType::DELETE;
                    }
                    if temp_pair.valid() && temp_pair.access() {
                        flag = HeadType::RENEW;
                    }
                } else {
                    break;
                }
                // 更新堆头
                match flag {
                    HeadType::DELETE => {
                        self.heap.pop();
                        continue;
                    }
                    HeadType::RENEW => {
                        let mut to_renew = self.heap.pop().unwrap();
                        let mut pair = self.pairs.get_mut(to_renew.id).unwrap();
                        let (first, second) = pair.get_vertex();

                        // 创建一个pair
                        let vertex1 = self.vertices[first].borrow();
                        let vertex2 = self.vertices[second].borrow();
                        pair.q_matrix = vertex1.q_v + vertex2.q_v;
                        pair.cal_best_point(&vertex1.coord, &vertex2.coord);
                        pair.cal_shrink_value();
                        pair.clear_access();

                        to_renew.value = OrderedFloat::from(pair.get_value());
                        self.heap.push(to_renew);

                        continue;
                    }
                    HeadType::NORMAL => {
                        break;
                    }
                }
            }
        }

        let mut real_idx : usize = 1;
        for step_idx in 0..self.vertices.len() {
            let v = self.vertices[step_idx].borrow();
            if !v.valid() {
                continue;
            }
            for (f_idx, pos) in v.faces.iter() {
                self.faces[*f_idx].indices[*pos] = real_idx;
            }
            real_idx += 1;
        }
        let mut face_num = 0;
        for f in self.faces.iter() {
            if f.valid() {
                face_num += 1;
            }
        }
        println!("new face num {}", face_num);
    }

    fn del_pair(&mut self, id : usize) {
        assert!( self.pairs[id].valid() );
        self.pairs[id].destroy();   // 将该pair置为无效

        // 获取旧顶点
        let (first, second) = self.pairs[id].get_vertex();
        assert!( first != second );
        let first_ptr = self.vertices[first].clone();
        let second_ptr = self.vertices[second].clone();
        let mut vertex1 = first_ptr.borrow_mut();
        let mut vertex2 = second_ptr.borrow_mut();

        // 回收旧顶点编号
        self.trash.push(first);
        self.trash.push(second);

        let new_pos = self.trash.pop().unwrap();

        // 获取用于存放新顶点的顶点实体
        let temp_pos = self.vertices.len() - 1;
        assert!( temp_pos != second && temp_pos != first);
        let temp_ptr = self.vertices[temp_pos].clone();
        let mut new_v = temp_ptr.borrow_mut();
        new_v.renew_state();
        assert!( new_v.faces.is_empty() );
        assert!( new_v.pairs.is_empty() );

        let best_point = self.pairs[id].get_best_point();
        new_v.coord.x = best_point.x;
        new_v.coord.y = best_point.y;
        new_v.coord.z = best_point.z;
        new_v.q_v = vertex1.q_v + vertex2.q_v;

        // 更新相关的面片
        for (idx, pos) in vertex1.faces.iter() {
            let valid = self.faces[*idx].valid();
            if valid {
                let both_in = self.faces[*idx].both_in(pos, &second);
                if both_in {
                    self.faces[*idx].destroy();
                    self.total -= 1;
                    continue;
                }
                self.faces[*idx].indices[*pos] = new_pos; // 更新面片的对应顶点坐标
                new_v.add_face(idx.clone(), pos.clone());
            }
        }
        for (idx, pos) in vertex2.faces.iter() {
            let valid = self.faces[*idx].valid();
            if valid {
                self.faces[*idx].indices[*pos] = new_pos;
                new_v.add_face(idx.clone(), pos.clone());
            }
        }


        // 更新相关的pair
        for (idx, pos) in vertex1.pairs.iter() {
            if *idx == id { 
                continue; 
            }
            if self.pairs[*idx].set_vertex(pos, new_pos) {
                new_v.pairs.push((idx.clone(), pos.clone()));
            } else {
                self.pairs[*idx].destroy();
            }
        }
        for (idx, pos) in vertex2.pairs.iter() {
            if *idx == id { 
                continue; 
            }
            if self.pairs[*idx].set_vertex(pos, new_pos) {
                new_v.pairs.push((idx.clone(), pos.clone()));
            } else {
                self.pairs[*idx].destroy();
            }

        }

        vertex1.destroy();
        vertex2.destroy();
        self.vertices.swap(new_pos, temp_pos);
    }

    pub fn save(&mut self, path : &Path) {
        let file = match File::create(&path) {
            Err(why) => panic!("couldn't create {}", why.to_string()),
            Ok(file) => file,
        };
        let mut file_writer = LineWriter::new(file);
        for v_ptr in self.vertices.iter() {
            let v = v_ptr.borrow();
            if v.valid() {
                file_writer.write_all(&v.get_string().into_bytes()).unwrap();
            }
        }
        for f in self.faces.iter() {
            if f.valid() {
                file_writer.write_all(&f.get_string().into_bytes()).unwrap();
            }
        }
        file_writer.flush().unwrap();
    }
}

#[derive(Debug)]
enum HeadType {
    DELETE,
    RENEW,
    NORMAL
}
