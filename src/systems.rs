use cgmath::num_traits::clamp;
use glium::glutin::dpi::PhysicalSize;
use hashbrown::HashMap;
use itertools::izip;
use rayon::iter::{IndexedParallelIterator, ParallelIterator};
use rayon::slice::{ParallelSlice, ParallelSliceMut};
use vecmath::{Vector2, vec2_add, vec2_len, vec2_normalized, vec2_scale, vec2_square_len, vec2_sub};

use crate::{AGENT_COUNT, ALIGNMENT_WEIGHT, CELL_SIZE, COHESION_WEIGHT, SEPARATION_WEIGHT, data::*};

// Moves boids forward.
pub fn forward_system(delta_time: f32, speed: f32, positions: &mut [Position], forwards: &[Forward]) {
    let real_speed = delta_time * speed;

    let forward_job = |position: &mut Position, forward: &Forward| {
        position.value = vec2_add(
            position.value,
            vec2_scale(forward.direction, real_speed)
        );
    };

    let chunk_size = AGENT_COUNT / rayon::current_num_threads();

    let pi = positions.par_chunks_mut(chunk_size);
    let fi = forwards.par_chunks(chunk_size);
    
    pi.zip(fi)
        .for_each(|(position_chunk, forward_chunk)| {
            for (position, forward) in izip!(position_chunk, forward_chunk) {
                forward_job(position, forward);
            }
        });
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

    let chunk_size = AGENT_COUNT / rayon::current_num_threads();

    let ti = transforms.par_chunks_mut(chunk_size);
    let pi = positions.par_chunks(chunk_size);
    let fi = forwards.par_chunks(chunk_size);

    ti.zip(pi).zip(fi)
        .for_each(|((transform_chunk, position_chunk), forward_chunk)| {
            for (t, p, f) in izip!(transform_chunk.iter_mut(), position_chunk.iter(), forward_chunk.iter()) {
                caluclate_transform_job(t, p, f)
            }
        });
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
fn bucket_alignment(boids: &[usize], forwards: &[Forward], cell_forward: &mut Forward) {
    for boid_id in boids {
        cell_forward.direction[0] += forwards[*boid_id].direction[0];
        cell_forward.direction[1] += forwards[*boid_id].direction[1];
    }

    cell_forward.direction = vec2_normalized(cell_forward.direction);
}

// Calculates an average position of each boid inside a cell
fn bucket_cohesion(boids: &[usize], positions: &[Position], cell_cohesion: &mut Position) {
    for boid_id in boids {
        cell_cohesion.value[0] += positions[*boid_id].value[0];
        cell_cohesion.value[1] += positions[*boid_id].value[1];
    }

    cell_cohesion.value = vec2_scale(cell_cohesion.value, 1.0 / boids.len() as f32);
}

// Calculate speparation for each boid inside a cell.
// This only checks each boid against boids from the same cell
// that can cause weird artefacts because the closest boid can be from other cell...
fn bucket_separation(boids: &[usize], positions: &[Position], separations: &mut [Forward]) {
    let mut nearest_index: usize;
    let mut min_distance: f32;
    let mut distance: f32;

    for boid_id in boids {

        nearest_index = 0;
        min_distance = f32::MAX;

        for neighbor_id in boids {
            // This if is very bad
            // TODO remove
            if boid_id.eq(neighbor_id) {
                continue;
            }

            distance = vec2_square_len(vec2_sub(
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

        min_distance = min_distance.sqrt();

        if min_distance != 0.0 {
            separations[*boid_id].direction = vec2_scale(
                separations[*boid_id].direction,
                clamp(1.0 / min_distance, 0.01, 100.0)
            );
        }
    }
}

pub fn boid_system(positions: &[Position], forwards: &mut[Forward]) {
    // This hashmap will be replaced by multi hash map
    let mut cells: HashMap<u32, Vec<usize>> = HashMap::with_capacity(AGENT_COUNT);

    // These array are big and storing cell data in them causes random placement.
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
    for (cell_id, boids) in &cells {
        bucket_alignment(boids, forwards, &mut cell_forwards[*cell_id as usize]);
        bucket_cohesion(boids, positions, &mut cell_cohesions[*cell_id as usize]);
        bucket_separation(boids, positions, &mut separations);
    }

    // Apply directions and cohesion
    for b in &cells {
        for agent_id in b.1 {
            let mut res = forwards[*agent_id].direction;

            // Cohesion
            let mut coh = vec2_sub(cell_cohesions[*b.0 as usize].value, positions[*agent_id].value);
            // Distance to cohesion point
            let d2c = vec2_len(coh);

            if d2c != 0.0 {
                coh = vec2_scale(coh, clamp(1.0 / d2c, 0.01, 100.0));
                coh = vec2_scale(coh, COHESION_WEIGHT);
                res = vec2_add(res, coh);
            }

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

    let threads = rayon::current_num_threads();
    let pi = positions.par_chunks_mut(AGENT_COUNT / threads);

    pi.for_each(|position_chunk| {
        for position in position_chunk {
            wrap_screen_job(position);
        }
    });
}