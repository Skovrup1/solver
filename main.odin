package main

import "core:fmt"
import "core:math"
import "core:math/linalg"
import "core:sort"
import "core:time"

import "vendor:raylib"

SCREEN_WIDTH :: 800
SCREEN_HEIGHT :: 600

TARGET_FPS :: 240
UPDATE_RATE :: 60
UPDATE_TIME :: time.Second / time.Duration(UPDATE_RATE)

GRAVITY :: 100
DAMPING :: 0.95
RESTITUTION :: 0.8

particles: #soa[dynamic]Particle
dynamics: #soa[dynamic]Dynamics2D
contacts: [dynamic]Contact

Vec2 :: [2]f32
Color :: raylib.Color

dot :: linalg.dot
normalize :: linalg.vector_normalize
length :: linalg.vector_length

Dynamics2D :: struct {
	inv_mass:     f32,
	damping:      f32,
	force:        Vec2,
	position:     Vec2,
	velocity:     Vec2,
	acceleration: Vec2,
	rotation:     f32,
}

Particle :: struct {
	size:     Vec2,
	color:    Color,
	dynamics: i32,
}

Contact :: struct {
	particle1_index: i32,
	particle2_index: i32,
	restitution:     f32,
	penetration:     f32,
	normal:          Vec2,
}

SatShape :: struct {
	center:   Vec2,
	vertices: [4]Vec2,
}

get_inv_mass :: proc(mass: f32) -> f32 {
	if mass == 0 {
		return 0
	} else {
		return 1 / mass
	}
}

make_particle :: proc(
	size: Vec2,
	color: Color = raylib.GRAY,
	inv_mass: f32 = 0,
	damping: f32 = DAMPING,
	force: Vec2 = {},
	position: Vec2 = {},
	velocity: Vec2 = {},
	acceleration: Vec2 = {},
	rotation: f32 = 0,
) -> Particle {
	dyn := Dynamics2D{inv_mass, damping, force, position, velocity, acceleration, rotation}
	index := i32(len(dynamics))
	append(&dynamics, dyn)
	return Particle{size, color, index}
}

create_sat_shape :: proc(p: Particle) -> SatShape {
	dyn := dynamics[p.dynamics]
	shape: SatShape

	half_size_x := p.size.x / 2
	half_size_y := p.size.y / 2
	shape.center = dyn.position + Vec2{half_size_x, half_size_y}

	cos_r := f32(math.cos(f64(dyn.rotation)))
	sin_r := f32(math.sin(f64(dyn.rotation)))

	corners := [4]Vec2 {
		{-half_size_x, -half_size_y},
		{half_size_x, -half_size_y},
		{half_size_x, half_size_y},
		{-half_size_x, half_size_y},
	}

	for corner, i in corners {
		rotated_x := corner.x * cos_r - corner.y * sin_r
		rotated_y := corner.x * sin_r + corner.y * cos_r
		shape.vertices[i] = shape.center + Vec2{rotated_x, rotated_y}
	}
	return shape
}

get_axes :: proc(shape: SatShape) -> [2]Vec2 {
	axes: [2]Vec2
	edge1 := shape.vertices[0] - shape.vertices[1]
	edge2 := shape.vertices[1] - shape.vertices[2]
	axes[0] = {-edge1.y, edge1.x}
	axes[1] = {-edge2.y, edge2.x}
	return axes
}

project :: proc(shape: SatShape, axis: Vec2) -> (min: f32, max: f32) {
	min = dot(shape.vertices[0], axis)
	max = min
	for i := 1; i < 4; i += 1 {
		p := dot(shape.vertices[i], axis)
		if p < min {
			min = p
		} else if p > max {
			max = p
		}
	}
	return
}

check_collisions :: proc() {
	clear(&contacts)
	for p1, i in particles {
		for p2, j in particles {
			if i >= j {continue}

			dyn1 := &dynamics[p1.dynamics]
			dyn2 := &dynamics[p2.dynamics]
			if dyn1.inv_mass == 0 && dyn2.inv_mass == 0 {continue}

			shape1 := create_sat_shape(p1)
			shape2 := create_sat_shape(p2)

			min_overlap := max(f32)
			collision_normal: Vec2

			axes1 := get_axes(shape1)
			axes2 := get_axes(shape2)
			all_axes := [?]Vec2{axes1[0], axes1[1], axes2[0], axes2[1]}

			separated := false
			for axis in all_axes {
				n_axis := normalize(axis)
				min1, max1 := project(shape1, n_axis)
				min2, max2 := project(shape2, n_axis)

				overlap := math.min(max1, max2) - math.max(min1, min2)
				if overlap <= 0 {
					separated = true
					break
				}
				if overlap < min_overlap {
					min_overlap = overlap
					collision_normal = n_axis
				}
			}

			if separated {continue}

			direction := shape2.center - shape1.center
			if dot(direction, collision_normal) < 0 {
				collision_normal = -collision_normal
			}

			contact := Contact{i32(i), i32(j), RESTITUTION, min_overlap, collision_normal}
			append(&contacts, contact)
		}
	}
}

integrate :: proc(time_step: f32) {
	for &dyn in dynamics {
		dyn.position += dyn.velocity * time_step
		dyn.acceleration = dyn.force * dyn.inv_mass
		dyn.velocity += dyn.acceleration * time_step
		dyn.velocity *= math.pow(f32(DAMPING), time_step)
		dyn.force = {}
	}
}

apply_gravity :: proc() {
	for &dyn in dynamics {
		mass: f32
		if dyn.inv_mass == 0 {
			mass = dyn.inv_mass
		} else {
			mass = 1 / dyn.inv_mass
		}
		dyn.force += {0, GRAVITY * mass}
	}
}

resolve_velocity :: proc(time_step: f32) {
	for contact in contacts {
		p1 := particles[contact.particle1_index]
		p2 := particles[contact.particle2_index]

		body1 := &dynamics[p1.dynamics]
		body2 := &dynamics[p2.dynamics]

		delta_velocity := body1.velocity - body2.velocity
		contact_normal := -contact.normal
		separating_velocity := dot(delta_velocity, contact_normal)

		if separating_velocity > 0 {continue}

		new_sep_velocity := -separating_velocity * contact.restitution

		acc_caused_velocity := body1.acceleration - body2.acceleration
		acc_caused_sep_velocity := dot(acc_caused_velocity, contact_normal) * time_step

		if acc_caused_sep_velocity < 0 {
			new_sep_velocity += contact.restitution * acc_caused_sep_velocity

			if new_sep_velocity < 0 {
				new_sep_velocity = 0
			}
		}

		velocity_change := new_sep_velocity - separating_velocity

		total_inv_mass := body1.inv_mass + body2.inv_mass

		if total_inv_mass == 0 {continue}

		impulse := velocity_change / total_inv_mass

		impulse_per_inv_mass := contact_normal * impulse

		body1.velocity += impulse_per_inv_mass * body1.inv_mass
		body2.velocity += impulse_per_inv_mass * -body2.inv_mass
	}
}

resolve_penetration :: proc() {
	for contact in contacts {
		if contact.penetration <= 0 {continue}

		p1 := particles[contact.particle1_index]
		p2 := particles[contact.particle2_index]

		body1 := &dynamics[p1.dynamics]
		body2 := &dynamics[p2.dynamics]

		total_inv_mass := body1.inv_mass + body2.inv_mass

		if total_inv_mass <= 0 {continue}

		move_per_inv_mass := contact.normal * (-contact.penetration / total_inv_mass)

		body1.position += move_per_inv_mass * body1.inv_mass
		body2.position += move_per_inv_mass * body2.inv_mass
	}
}

calculate_separating_velocity :: proc(contact: Contact) -> f32 {
	p1 := particles[contact.particle1_index]
	p2 := particles[contact.particle2_index]

	body1 := dynamics[p1.dynamics]
	body2 := dynamics[p2.dynamics]

	delta_velocity := body1.velocity - body2.velocity
	contact_normal := -contact.normal
	separating_velocity := dot(delta_velocity, contact_normal)

	return separating_velocity
}

resolve_contacts :: proc(time_step: f32) {
	contact_sort :: proc(fst: Contact, snd: Contact) -> int {
		a := calculate_separating_velocity(fst)
		b := calculate_separating_velocity(snd)

		if a < b {return -1}
		if a > b {return 1}
		return 0
	}
	sort.quick_sort_proc(contacts[:], contact_sort)

	resolve_velocity(time_step)
	resolve_penetration()
}

update :: proc(time_step: f32) {
	apply_gravity()

	check_collisions()

	resolve_contacts(time_step)

	integrate(time_step)
}

draw_particles :: proc() {
	for particle in particles {
		dyn := dynamics[particle.dynamics]
		rect := raylib.Rectangle{dyn.position.x, dyn.position.y, particle.size.x, particle.size.y}
		origin := Vec2{0, 0} // rotate around top-left corner
		raylib.DrawRectanglePro(rect, origin, dyn.rotation * f32(180 / math.PI), particle.color)
	}
}

draw :: proc() {
	raylib.BeginDrawing()
	raylib.ClearBackground(raylib.RAYWHITE)
	draw_particles()
	raylib.EndDrawing()
}

hard_constraint_scene :: proc() {
	p1 := make_particle(
		size = {50, 50},
		position = {SCREEN_WIDTH / 2.0 - 25, 100},
		inv_mass = get_inv_mass(10),
		color = raylib.MAROON,
	)
	append(&particles, p1)

	p2 := make_particle(
		size = {50, 50},
		position = {SCREEN_WIDTH / 2.0 - 25, 200},
		inv_mass = get_inv_mass(10),
		color = raylib.DARKBLUE,
	)
	append(&particles, p2)

	p3 := make_particle(
		size = {50, 50},
		position = {SCREEN_WIDTH / 2.0 - 25, 300},
		inv_mass = get_inv_mass(10),
		color = raylib.GOLD,
	)
	append(&particles, p3)

	floor := make_particle(size = {SCREEN_WIDTH, 50}, position = {0, SCREEN_HEIGHT - 50})
	append(&particles, floor)
}

setup :: proc() {
	hard_constraint_scene()
}

main :: proc() {
	raylib.SetTraceLogLevel(raylib.TraceLogLevel.WARNING)
	raylib.InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics Solver")
	defer raylib.CloseWindow()
	raylib.SetTargetFPS(TARGET_FPS)

	setup()

	accumulator: time.Duration = 0
	for !raylib.WindowShouldClose() {
		accumulator += time.Duration(f64(raylib.GetFrameTime()) * f64(time.Second))
		for accumulator >= UPDATE_TIME {
			update_time_seconds := cast(f32)time.duration_seconds(UPDATE_TIME)
			update(update_time_seconds)
			accumulator -= UPDATE_TIME
		}

		draw()
	}
}
