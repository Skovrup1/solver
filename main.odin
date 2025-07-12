package main

import "core:fmt"
import "core:math"
import "core:math/linalg"
import "core:math/rand"
import "core:time"

import "vendor:raylib"

Vec2 :: [2]f32
Vec3 :: [3]f32

dot :: linalg.dot

Color :: raylib.Color

SCREEN_WIDTH :: 800
SCREEN_HEIGHT :: 600

TARGET_FPS :: 240

UPDATE_RATE :: 60 // Hz
UPDATE_TIME :: time.Second / time.Duration(UPDATE_RATE) // ns

GRAVITY :: 700
DAMPING :: 0.95

particles: #soa[dynamic]Particle
dynamics: #soa[dynamic]Dynamics2D

//rng := rand.create(0)

Dynamics2D :: struct {
	inv_mass:     f32, // 1 / mass
	damping:      f32,
	force:        Vec2,
	position:     Vec2,
	velocity:     Vec2,
	acceleration: Vec2,
}

get_inv_mass :: proc(mass: f32) -> f32 {
	return 1 / mass
}

set_mass :: proc(dyn: ^Dynamics2D, mass: f32) {
	dyn.inv_mass = 1 / mass
}

Particle :: struct {
	size:     Vec2,
	color:    Color,
	dynamics: i32, // index into dynamics
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
) -> Particle {
	dyn := Dynamics2D{inv_mass, damping, force, position, velocity, acceleration}
	index := i32(len(dynamics))
	append(&dynamics, dyn)

	return Particle{size, color, index}
}

draw_particles :: proc() {
	for particle in particles {
		dyn := dynamics[particle.dynamics]
		raylib.DrawRectangleV(dyn.position, particle.size, particle.color)
	}
}

integrate :: proc(time_step: f32) {
	for &dyn in dynamics {
		dyn.position += dyn.velocity * time_step

		dyn.acceleration = dyn.force * dyn.inv_mass

		dyn.velocity += dyn.acceleration * time_step
		dyn.velocity *= math.pow(dyn.damping, time_step)

		dyn.force = {}
	}
}

apply_gravity :: proc() {
	for &dyn in dynamics {
		dyn.force += {0, GRAVITY}
	}
}

apply_drag :: proc() {
	for &dyn in dynamics {
		k1: f32 = 0
		k2: f32 = 0.001
		speed := linalg.vector_length(dyn.velocity)

		drag_coeff := k1 * speed + k2 * speed * speed

		norm_velocity := linalg.vector_normalize(dyn.velocity)
		drag_force := norm_velocity * -drag_coeff

		dyn.force += drag_force
	}
}

apply_springs :: proc() {

}

update :: proc(time_step: f32) {
	apply_gravity()

	apply_springs()
	//apply_drag()

	integrate(time_step)
}

draw :: proc() {
	raylib.BeginDrawing()

	raylib.ClearBackground(raylib.RAYWHITE)

	draw_particles()

	raylib.EndDrawing()
}

main :: proc() {
	raylib.SetTraceLogLevel(raylib.TraceLogLevel.WARNING)
	raylib.InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics solver")
	defer raylib.CloseWindow()

	raylib.SetTargetFPS(TARGET_FPS)

	//floor_size := Vec2{SCREEN_WIDTH, 100}
	//floor_pos := Vec2{0, 500}
	//append(&particles, make_particle(floor_size, floor_pos))

	for i in 0 ..< 16 {
		particle_size := Vec2{10, 10}
		particle_pos := Vec2{SCREEN_WIDTH / 2.0, 500}

		x_vel := f32(i) * 12
		x_vel *= i % 2 == 0 ? 1 : -1
		particle_vel := Vec2{x_vel, -800}

		append(
			&particles,
			make_particle(
				size = particle_size,
				position = particle_pos,
				velocity = particle_vel,
				inv_mass = get_inv_mass(1),
				color = raylib.MAROON,
			),
		)
	}

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
