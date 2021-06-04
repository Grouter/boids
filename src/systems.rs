use crossbeam::thread;
use hashbrown::HashMap;
use vecmath::{Vector2, vec2_add, vec2_len, vec2_normalized, vec2_scale, vec2_sub};

use crate::{AGENT_COUNT, ALIGNMENT_WEIGHT, CELL_SIZE, COHESION_WEIGHT, SEPARATION_WEIGHT, data::*};

const THREAD_COUNT: usize = 8;
const CHUNK_SIZE: usize = AGENT_COUNT / THREAD_COUNT;

// TODO create handlers for chunk iterations?
// There is a lot of duplicate code for chunk iteration...

pub fn forward_system(dt: f32, speed: f32, positions: &mut [Position], forwards: &[Forward]) {
    thread::scope(|s| {
        positions
            .chunks_mut(CHUNK_SIZE)
            .enumerate()
            .for_each(|(i, chunk)| {
                s.spawn(move |_| {
                    let offset = CHUNK_SIZE * i;
                    let mut index: usize;

                    for (local_i, position) in chunk.iter_mut().enumerate() {
                        index = offset + local_i;

                        position.value = vec2_add(
                            position.value, 
                            vec2_scale(forwards[index].direction, dt * speed)
                        );
                    }
                });
            })
    }).expect("Thread panic");
}

pub fn caluclate_transform_system(transforms: &mut [Transform], positions: &[Position]) {
    thread::scope(|s| {
        transforms
            .chunks_mut(CHUNK_SIZE)
            .enumerate()
            .for_each(|(i, chunk)| {
                s.spawn(move |_| {
                    let offset = CHUNK_SIZE * i;
                    let mut index: usize;

                    for (local_i, transform) in chunk.iter_mut().enumerate() {
                        index = offset + local_i;

                        let t = [
                            [1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0],
                            [positions[index].value[0], positions[index].value[1], 0.0, 1.0],
                        ];

                        transform.transform = t;
                    }
                });
            });
    }).expect("Thread panic");
} 

fn vec2_normalized_safe(v: Vector2<f32>) -> Vector2<f32> {
    let l = vec2_len(v);

    if l == 0.0 {
        return [0.0, 0.0];
    }

    [v[0] / l, v[1] / l]
}

// http://www.beosil.com/download/CollisionDetectionHashing_VMV03.pdf
fn hash(position: &Position) -> u32 {
    const P1: u32 = 73856093;
    const P2: u32 = 19349663;
    //const p3: u32 = 83492791;

    let cell_x = (position.value[0] / CELL_SIZE).floor();
    let cell_y = (position.value[1] / CELL_SIZE).floor();

    let h = (cell_x as u32 * P1) ^ (cell_y as u32 * P2);

    h % AGENT_COUNT as u32
}


fn bucket_alignment(bucket: (&u32, &Vec<usize>), forwards: &[Forward], cell_forward: &mut Forward) {
    for agent_id in bucket.1 {
        cell_forward.direction[0] += forwards[*agent_id].direction[0];
        cell_forward.direction[1] += forwards[*agent_id].direction[1];
    }

    cell_forward.direction = vec2_normalized(cell_forward.direction);
}

fn bucket_cohesion(bucket: (&u32, &Vec<usize>), positions: &[Position], cell_cohesion: &mut Position) {
    for agent_id in bucket.1 {
        cell_cohesion.value[0] += positions[*agent_id].value[0];
        cell_cohesion.value[1] += positions[*agent_id].value[1];
    }

    cell_cohesion.value = vec2_scale(cell_cohesion.value, 1.0 / bucket.1.len() as f32);
}

fn bucket_separation(bucket: (&u32, &Vec<usize>), positions: &[Position], separations: &mut [Forward]) {
    let mut nearest_index: usize;
    let mut min_distance: f32;
    let mut distance: f32;

    for boid_id in bucket.1 {

        nearest_index = 0;
        min_distance = f32::MAX;
        
        for neighbor_id in bucket.1 {
            // This if is very bad
            // TODO remove
            if boid_id.eq(neighbor_id) {
                continue;
            }

            distance = vec2_len(vec2_sub(
                positions[*boid_id].value, 
                positions[*neighbor_id].value
            ));

            if distance < min_distance {
                min_distance = distance;
                nearest_index = *neighbor_id;
            }
        }

        separations[*boid_id].direction = vec2_normalized_safe(vec2_sub(
            positions[*boid_id].value,
            positions[nearest_index].value, 
        ));
    }
}

pub fn boid_system(positions: &[Position], forwards: &mut[Forward]) {
    let mut cells: HashMap<u32, Vec<usize>> = HashMap::with_capacity(AGENT_COUNT);
    
    let mut cell_forwards: Vec<Forward> = vec![Forward { direction: [0.0, 0.0] }; AGENT_COUNT];
    let mut cell_cohesions: Vec<Position> = vec![Position { value: [0.0, 0.0] }; AGENT_COUNT];
    
    let mut separations: Vec<Forward> = vec![Forward { direction: [0.0, 0.0] }; AGENT_COUNT];

    // Divide all agents into separate cells to reduce calculations 
    for (i, position) in positions.iter().enumerate() {
        let h = hash(position);

        if cells.contains_key(&h) {
            cells.get_mut(&h).unwrap().push(i);
        }
        else {
            // This is really temporary... until I make my own
            // Multi Hash Map
            let mut v = Vec::with_capacity(1000);
            v.push(i);
            cells.insert(h, v);
        }
    }

    // Calculate general direction for each cell
    for b in &cells {
        thread::scope(|s| {
            s.spawn(|_| {
                bucket_alignment(b, forwards, &mut cell_forwards[*b.0 as usize]);
            });
            s.spawn(|_| {
                bucket_cohesion(b, positions, &mut cell_cohesions[*b.0 as usize]);
            });
            s.spawn(|_| {
                bucket_separation(b, positions, &mut separations);
            });
        }).expect("Thread paniced");
    }

    // TODO threading?
    // Apply directions and cohesion
    for b in &cells {
        for agent_id in b.1 {
            let mut res = forwards[*agent_id].direction;
            
            // Cohesion
            let mut coh = vec2_sub(cell_cohesions[*b.0 as usize].value, positions[*agent_id].value);
            coh = vec2_normalized_safe(coh);
            coh = vec2_scale(coh, COHESION_WEIGHT);
            res = vec2_add(res, coh);

            // Separation
            separations[*agent_id].direction = vec2_scale(
                separations[*agent_id].direction, 
                SEPARATION_WEIGHT
            );
            res = vec2_add(res, separations[*agent_id].direction);
            
            cell_forwards[*b.0 as usize].direction = vec2_scale(
                cell_forwards[*b.0 as usize].direction, 
                ALIGNMENT_WEIGHT
            );
            res = vec2_add(res, cell_forwards[*b.0 as usize].direction);

            forwards[*agent_id].direction = vec2_normalized(res);
        }
    }
}
