use bisetmap::BisetMap;
use vecmath::{Vector2, vec2_add, vec2_len, vec2_mul, vec2_normalized, vec2_scale, vec2_sub};

use crate::{AGENT_COUNT, CELL_SIZE, data::*};

pub fn forward_system(dt: f32, speed: f32, positions: &mut [Position], forwards: &[Forward]) {
    for i in 0..AGENT_COUNT {
        positions[i].value = vec2_add(
            positions[i].value, 
            vec2_mul(forwards[i].direction, [dt * speed, dt * speed])
        );
    }
}

pub fn caluclate_transform_system(transforms: &mut [Transform], positions: &[Position]) {
    for i in 0..AGENT_COUNT {
        let t = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [positions[i].value[0], positions[i].value[1], 0.0, 1.0],
        ];

        transforms[i].transform = t;
    }
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


fn bucket_alignment(bucket: &(u32, Vec<usize>), forwards: &[Forward], cell_forward: &mut Forward) {
    for agent_id in &bucket.1 {
        cell_forward.direction[0] += forwards[*agent_id].direction[0];
        cell_forward.direction[1] += forwards[*agent_id].direction[1];
    }

    cell_forward.direction = vec2_normalized(cell_forward.direction);
}

fn bucket_cohesion(bucket: &(u32, Vec<usize>), positions: &[Position], cell_cohesion: &mut Position) {
    for agent_id in &bucket.1 {
        cell_cohesion.value[0] += positions[*agent_id].value[0];
        cell_cohesion.value[1] += positions[*agent_id].value[1];
    }

    cell_cohesion.value = vec2_scale(cell_cohesion.value, 1.0 / bucket.1.len() as f32);
}

fn bucket_separation(bucket: &(u32, Vec<usize>), positions: &[Position], separations: &mut [Forward]) {
    let mut nearest_index: usize;
    let mut min_distance: f32;
    let mut distance: f32;

    for boid_id in &bucket.1 {

        nearest_index = 0;
        min_distance = f32::MAX;
        
        for neighbor_id in &bucket.1 {
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
    let cells: BisetMap<u32, usize> = BisetMap::with_capacity(AGENT_COUNT);
    
    let mut cell_forwards: [Forward; AGENT_COUNT] = [Forward { direction: [0.0, 0.0] }; AGENT_COUNT];
    let mut cell_cohesions: [Position; AGENT_COUNT] = [Position { value: [0.0, 0.0] }; AGENT_COUNT];
    let mut separations: [Forward; AGENT_COUNT] = [Forward { direction: [0.0, 0.0] }; AGENT_COUNT];

    // Divide all agents into separate cells to reduce calculations 
    for (i, position) in positions.iter().enumerate() {
        let h = hash(position);
        cells.insert(h, i);
    }

    // TODO threading
    // Calculate general direction for each cell
    let buckets = cells.collect();
    for b in &buckets {
        bucket_alignment(b, forwards, &mut cell_forwards[b.0 as usize]);
        bucket_cohesion(b, positions, &mut cell_cohesions[b.0 as usize]);
        bucket_separation(b, positions, &mut separations);
    }

    // TODO threading?
    // Apply directions and cohesion
    for b in &buckets {
        for agent_id in &b.1 {
            let mut res = forwards[*agent_id].direction;
            
            // Cohesion
            let mut coh = vec2_sub(cell_cohesions[b.0 as usize].value, positions[*agent_id].value);
            coh = vec2_normalized_safe(coh);
            coh = vec2_scale(coh, 0.7);
            res = vec2_add(res, coh);

            // Separation
            separations[*agent_id].direction = vec2_scale(separations[*agent_id].direction, 0.8);
            res = vec2_add(res, separations[*agent_id].direction);
            
            //cell_forwards[b.0 as usize].direction = vec2_scale(cell_forwards[b.0 as usize].direction, 1.0);
            res = vec2_add(res, cell_forwards[b.0 as usize].direction);

            forwards[*agent_id].direction = vec2_normalized(res);
        }
    }
}
