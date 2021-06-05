use cgmath::num_traits::clamp;
use crossbeam::thread;
use glium::glutin::dpi::PhysicalSize;
use hashbrown::HashMap;
use vecmath::{Vector2, vec2_add, vec2_len, vec2_normalized, vec2_scale, vec2_sub};

use crate::{AGENT_COUNT, ALIGNMENT_WEIGHT, CELL_SIZE, COHESION_WEIGHT, SEPARATION_WEIGHT, data::*};

const THREAD_COUNT: usize = 8;
const CHUNK_SIZE: usize = AGENT_COUNT / THREAD_COUNT;

// Moves boids forward.
pub fn forward_system(delta_time: f32, speed: f32, positions: &mut [Position], forwards: &[Forward]) {
    let real_speed = delta_time * speed;
    
    let forward_job = |position: &mut Position, forward: &Forward| {
        position.value = vec2_add(
            position.value,
            vec2_scale(forward.direction, real_speed)
        );
    };

    let pi = positions.chunks_mut(CHUNK_SIZE);
    let fi = forwards.chunks(CHUNK_SIZE);

    thread::scope(|s| {
        pi.zip(fi).for_each(|(pc, fc)| {
            s.spawn(move |_| {
                pc.iter_mut()
                    .zip(fc.iter())
                    .for_each(
                        |(position, forward)| 
                            forward_job(position, forward)
                    );
            });
        });
    }).expect("Thread panic");
}

pub fn caluclate_transform_system(transforms: &mut [Transform], positions: &[Position], forwards: &[Forward]) {

    let caluclate_transform_job = |transform: &mut Transform, position: &Position, forward: &Forward| {
        let cos = forward.direction[0];
        let sin = -forward.direction[1];

        let t = [
            [cos,-sin, 0.0, 0.0],
            [sin, cos, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [position.value[0], position.value[1], 0.0, 1.0],
        ];

        transform.transform = t;
    };

    let ti = transforms.chunks_mut(CHUNK_SIZE);
    let pi = positions.chunks(CHUNK_SIZE);
    let fi = forwards.chunks(CHUNK_SIZE);

    thread::scope(|s| {
        ti.zip(pi).zip(fi).for_each(|((tc, pc), fc)| {
            s.spawn(move |_| {
                tc.iter_mut()
                .zip(pc.iter())
                .zip(fc.iter())
                .for_each(
                    |((transform, position), forward)| 
                        caluclate_transform_job(transform, position, forward)
                );
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

// Calculates an average direction of each boid inside a cell
fn bucket_alignment(bucket: (&u32, &Vec<usize>), forwards: &[Forward], cell_forward: &mut Forward) {
    for agent_id in bucket.1 {
        cell_forward.direction[0] += forwards[*agent_id].direction[0];
        cell_forward.direction[1] += forwards[*agent_id].direction[1];
    }

    cell_forward.direction = vec2_normalized(cell_forward.direction);
}

// Calculates an average position of each boid inside a cell
fn bucket_cohesion(bucket: (&u32, &Vec<usize>), positions: &[Position], cell_cohesion: &mut Position) {
    for agent_id in bucket.1 {
        cell_cohesion.value[0] += positions[*agent_id].value[0];
        cell_cohesion.value[1] += positions[*agent_id].value[1];
    }

    cell_cohesion.value = vec2_scale(cell_cohesion.value, 1.0 / bucket.1.len() as f32);
}

// Calculate speparation for each boid inside a cell.
// This only checks each boid against boids from the same cell
// that can cause weird artefacts because the closest boid can be from other cell...
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

        
        if min_distance != 0.0 {
            separations[*boid_id].direction = vec2_scale(
                separations[*boid_id].direction, 
                clamp(1.0 / min_distance, 0.01, 1000.0)
            );
        }
    }
}

pub fn boid_system(positions: &[Position], forwards: &mut[Forward]) {
    // This hashmap will be replaced by multi hash map
    let mut cells: HashMap<u32, Vec<usize>> = HashMap::with_capacity(AGENT_COUNT);
    
    let mut cell_forwards: Vec<Forward> = Vec::new();
    cell_forwards.resize(AGENT_COUNT, Forward { direction: [0.0, 0.0] });

    let mut cell_cohesions: Vec<Position> = Vec::new();
    cell_cohesions.resize(AGENT_COUNT, Position { value: [0.0, 0.0] });
    
    let mut separations: Vec<Forward> = Vec::new();
    separations.resize(AGENT_COUNT, Forward { direction: [0.0, 0.0] });

    // Divide all agents into separate cells to reduce calculations 
    for (i, position) in positions.iter().enumerate() {
        let h = hash(position);

        if let Some(bucket) = cells.get_mut(&h) {
            bucket.push(i);
        }
        else {
            // This is really temporary... until I make my own multi hash map
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

// Wraps boid arund the screen.
// If boid will try to go out of a screen, it will appear on the other side.
pub fn wrap_screen_system(positions: &mut[Position], display: &PhysicalSize<u32>) {

    let wrap_screen_job = |position: &mut Position| {
        if position.value[0] < 0.0 {
            position.value[0] = display.width as f32;
        }
        else if position.value[0] > display.width as f32 {
            position.value[0] = 0.0;
        }

        if position.value[1] < 0.0 {
            position.value[1] = display.height as f32;
        }
        else if position.value[1] > display.height as f32 {
            position.value[1] = 0.0;
        }
    };

    let pi = positions.chunks_mut(CHUNK_SIZE);

    thread::scope(|s| {
        pi.for_each(|pc| {
            s.spawn(move |_| {
                pc.iter_mut()
                    .for_each(
                        |position| 
                            wrap_screen_job(position)
                    );
            });
        });
    }).expect("Thread panic");
}