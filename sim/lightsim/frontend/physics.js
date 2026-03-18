/**
 * @file physics.js
 * @brief Rigid body physics engine for Lightsim
 *
 * Lightweight rigid body simulation with:
 * - Fixed-timestep integration (semi-implicit Euler)
 * - Sphere, box, and plane collision primitives
 * - Impulse-based collision response with friction & restitution
 * - Multiple body support (static, kinematic, dynamic)
 */

// ============================================================================
// Vector3 math (minimal, no dependency)
// ============================================================================

class Vec3 {
    constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
    set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; }
    copy(v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
    clone() { return new Vec3(this.x, this.y, this.z); }
    add(v) { return new Vec3(this.x + v.x, this.y + v.y, this.z + v.z); }
    sub(v) { return new Vec3(this.x - v.x, this.y - v.y, this.z - v.z); }
    scale(s) { return new Vec3(this.x * s, this.y * s, this.z * s); }
    addScaled(v, s) { return new Vec3(this.x + v.x * s, this.y + v.y * s, this.z + v.z * s); }
    dot(v) { return this.x * v.x + this.y * v.y + this.z * v.z; }
    cross(v) {
        return new Vec3(
            this.y * v.z - this.z * v.y,
            this.z * v.x - this.x * v.z,
            this.x * v.y - this.y * v.x
        );
    }
    lengthSq() { return this.x * this.x + this.y * this.y + this.z * this.z; }
    length() { return Math.sqrt(this.lengthSq()); }
    normalize() {
        const len = this.length();
        if (len < 1e-10) return new Vec3();
        return this.scale(1 / len);
    }
}

// ============================================================================
// Collision shapes
// ============================================================================

const SHAPE_SPHERE = 0;
const SHAPE_PLANE = 1;
const SHAPE_BOX = 2;

class SphereShape {
    constructor(radius) {
        this.type = SHAPE_SPHERE;
        this.radius = radius;
    }
}

class PlaneShape {
    constructor(normal = new Vec3(0, 1, 0), offset = 0) {
        this.type = SHAPE_PLANE;
        this.normal = normal.normalize();
        this.offset = offset;
    }
}

class BoxShape {
    constructor(halfExtents) {
        this.type = SHAPE_BOX;
        this.halfExtents = halfExtents; // Vec3
    }
}

// ============================================================================
// Rigid body
// ============================================================================

const BODY_STATIC = 0;
const BODY_DYNAMIC = 1;
const BODY_KINEMATIC = 2;

class RigidBody {
    constructor(options = {}) {
        this.position = options.position ? options.position.clone() : new Vec3();
        this.velocity = options.velocity ? options.velocity.clone() : new Vec3();
        this.force = new Vec3();

        this.mass = options.mass !== undefined ? options.mass : 1;
        this.invMass = this.mass > 0 ? 1 / this.mass : 0;

        this.restitution = options.restitution !== undefined ? options.restitution : 0.5;
        this.friction = options.friction !== undefined ? options.friction : 0.4;
        this.linearDamping = options.linearDamping !== undefined ? options.linearDamping : 0.01;

        this.shape = options.shape || new SphereShape(1);
        this.type = options.type !== undefined ? options.type : BODY_DYNAMIC;

        if (this.type === BODY_STATIC || this.type === BODY_KINEMATIC) {
            this.invMass = 0;
        }

        // Angular velocity (simplified - no full rotation matrix, just spin)
        this.angularVelocity = new Vec3();
        this.angularDamping = 0.05;

        // User data for linking to Three.js mesh
        this.userData = options.userData || null;
        this.id = RigidBody._nextId++;
    }

    applyForce(f) {
        this.force = this.force.add(f);
    }

    applyImpulse(impulse) {
        if (this.invMass === 0) return;
        this.velocity = this.velocity.addScaled(impulse, this.invMass);
    }
}
RigidBody._nextId = 0;

// ============================================================================
// Contact / collision result
// ============================================================================

class Contact {
    constructor(bodyA, bodyB, normal, depth, point) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.normal = normal;  // points from A to B
        this.depth = depth;    // penetration depth
        this.point = point;    // contact point in world space
    }
}

// ============================================================================
// Collision detection
// ============================================================================

function detectSpherePlane(sphere, plane) {
    const dist = sphere.position.dot(plane.shape.normal) - plane.shape.offset;
    const penetration = sphere.shape.radius - dist;
    if (penetration > 0) {
        const point = sphere.position.sub(plane.shape.normal.scale(dist));
        return new Contact(sphere, plane, plane.shape.normal, penetration, point);
    }
    return null;
}

function detectSphereSphere(a, b) {
    const diff = b.position.sub(a.position);
    const dist = diff.length();
    const minDist = a.shape.radius + b.shape.radius;
    if (dist < minDist && dist > 1e-6) {
        const normal = diff.scale(1 / dist);
        const depth = minDist - dist;
        const point = a.position.addScaled(normal, a.shape.radius - depth * 0.5);
        return new Contact(a, b, normal, depth, point);
    }
    return null;
}

function detectSphereBox(sphere, box) {
    // Transform sphere center to box local space (assumes axis-aligned box at box.position)
    const local = sphere.position.sub(box.position);
    const he = box.shape.halfExtents;

    // Clamp to box surface
    const closest = new Vec3(
        Math.max(-he.x, Math.min(he.x, local.x)),
        Math.max(-he.y, Math.min(he.y, local.y)),
        Math.max(-he.z, Math.min(he.z, local.z))
    );

    const diff = local.sub(closest);
    const distSq = diff.lengthSq();
    const r = sphere.shape.radius;

    if (distSq < r * r && distSq > 1e-10) {
        const dist = Math.sqrt(distSq);
        const normal = diff.scale(1 / dist);
        const depth = r - dist;
        const worldClosest = box.position.add(closest);
        return new Contact(sphere, box, normal, depth, worldClosest);
    }

    // Check if sphere center is inside box
    if (distSq < 1e-10) {
        // Find closest face
        const faces = [
            { n: new Vec3(1, 0, 0), d: he.x - local.x },
            { n: new Vec3(-1, 0, 0), d: he.x + local.x },
            { n: new Vec3(0, 1, 0), d: he.y - local.y },
            { n: new Vec3(0, -1, 0), d: he.y + local.y },
            { n: new Vec3(0, 0, 1), d: he.z - local.z },
            { n: new Vec3(0, 0, -1), d: he.z + local.z },
        ];
        let minFace = faces[0];
        for (const f of faces) {
            if (f.d < minFace.d) minFace = f;
        }
        return new Contact(sphere, box, minFace.n, r + minFace.d,
            box.position.add(closest));
    }

    return null;
}

function detectCollision(a, b) {
    const ta = a.shape.type;
    const tb = b.shape.type;

    if (ta === SHAPE_SPHERE && tb === SHAPE_PLANE) return detectSpherePlane(a, b);
    if (ta === SHAPE_SPHERE && tb === SHAPE_SPHERE) return detectSphereSphere(a, b);
    if (ta === SHAPE_SPHERE && tb === SHAPE_BOX) return detectSphereBox(a, b);

    // Reverse order
    if (tb === SHAPE_SPHERE && ta === SHAPE_PLANE) {
        const c = detectSpherePlane(b, a);
        if (c) { const t = c.bodyA; c.bodyA = c.bodyB; c.bodyB = t; c.normal = c.normal.scale(-1); }
        return c;
    }
    if (tb === SHAPE_SPHERE && ta === SHAPE_BOX) {
        const c = detectSphereBox(b, a);
        if (c) { const t = c.bodyA; c.bodyA = c.bodyB; c.bodyB = t; c.normal = c.normal.scale(-1); }
        return c;
    }

    return null;
}

// ============================================================================
// Collision resolution
// ============================================================================

function resolveContact(contact) {
    const { bodyA, bodyB, normal, depth } = contact;

    // Skip if both are immovable
    if (bodyA.invMass === 0 && bodyB.invMass === 0) return;

    // Separate bodies (positional correction)
    const totalInvMass = bodyA.invMass + bodyB.invMass;
    const correction = normal.scale(depth * 0.8 / totalInvMass); // 80% correction to avoid jitter
    if (bodyA.type === BODY_DYNAMIC) {
        bodyA.position = bodyA.position.sub(correction.scale(bodyA.invMass));
    }
    if (bodyB.type === BODY_DYNAMIC) {
        bodyB.position = bodyB.position.add(correction.scale(bodyB.invMass));
    }

    // Relative velocity
    const relVel = bodyB.velocity.sub(bodyA.velocity);
    const velAlongNormal = relVel.dot(normal);

    // Don't resolve if separating
    if (velAlongNormal > 0) return;

    // Restitution (use minimum)
    const e = Math.min(bodyA.restitution, bodyB.restitution);

    // Kill bounce for very small velocities
    const effectiveE = Math.abs(velAlongNormal) < 1.0 ? 0 : e;

    // Impulse magnitude
    const j = -(1 + effectiveE) * velAlongNormal / totalInvMass;
    const impulse = normal.scale(j);

    bodyA.applyImpulse(impulse.scale(-1));
    bodyB.applyImpulse(impulse);

    // Friction
    let tangent = relVel.sub(normal.scale(velAlongNormal));
    const tangentLenSq = tangent.lengthSq();
    if (tangentLenSq > 1e-10) {
        tangent = tangent.scale(1 / Math.sqrt(tangentLenSq));
        const mu = Math.sqrt(bodyA.friction * bodyB.friction); // geometric mean
        const jt = -relVel.dot(tangent) / totalInvMass;

        // Coulomb friction: clamp tangential impulse
        const maxFriction = mu * Math.abs(j);
        const clampedJt = Math.max(-maxFriction, Math.min(maxFriction, jt));
        const frictionImpulse = tangent.scale(clampedJt);

        bodyA.applyImpulse(frictionImpulse.scale(-1));
        bodyB.applyImpulse(frictionImpulse);
    }

    // Transfer angular velocity from contact
    if (bodyA.type === BODY_DYNAMIC && bodyA.shape.type === SHAPE_SPHERE) {
        const r = bodyA.shape.radius;
        if (r > 0) {
            bodyA.angularVelocity = bodyA.angularVelocity.add(
                normal.cross(impulse.scale(-1)).scale(1 / (0.4 * bodyA.mass * r * r))
            );
        }
    }
    if (bodyB.type === BODY_DYNAMIC && bodyB.shape.type === SHAPE_SPHERE) {
        const r = bodyB.shape.radius;
        if (r > 0) {
            bodyB.angularVelocity = bodyB.angularVelocity.add(
                normal.cross(impulse).scale(1 / (0.4 * bodyB.mass * r * r))
            );
        }
    }
}

// ============================================================================
// Physics World
// ============================================================================

class PhysicsWorld {
    constructor(options = {}) {
        this.gravity = options.gravity || new Vec3(0, -9.81, 0);
        this.bodies = [];
        this.fixedTimeStep = options.fixedTimeStep || 1 / 60;
        this.maxSubSteps = options.maxSubSteps || 3;
        this.accumulator = 0;
        this.pixelScale = options.pixelScale || 1; // px per meter (for unit conversion)
    }

    addBody(body) {
        this.bodies.push(body);
        return body;
    }

    removeBody(body) {
        const idx = this.bodies.indexOf(body);
        if (idx >= 0) this.bodies.splice(idx, 1);
    }

    step(dt) {
        this.accumulator += dt;
        let steps = 0;

        while (this.accumulator >= this.fixedTimeStep && steps < this.maxSubSteps) {
            this._stepFixed(this.fixedTimeStep);
            this.accumulator -= this.fixedTimeStep;
            steps++;
        }
    }

    _stepFixed(dt) {
        const scaledGravity = this.gravity.scale(this.pixelScale);

        // Apply forces & integrate
        for (const body of this.bodies) {
            if (body.type !== BODY_DYNAMIC) continue;

            // Gravity
            body.applyForce(scaledGravity.scale(body.mass));

            // Semi-implicit Euler integration
            const accel = body.force.scale(body.invMass);
            body.velocity = body.velocity.add(accel.scale(dt));

            // Linear damping
            body.velocity = body.velocity.scale(1 - body.linearDamping);

            // Angular damping
            body.angularVelocity = body.angularVelocity.scale(1 - body.angularDamping);

            // Update position
            body.position = body.position.addScaled(body.velocity, dt);

            // Clear forces
            body.force = new Vec3();
        }

        // Collision detection & resolution
        for (let i = 0; i < this.bodies.length; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                const a = this.bodies[i];
                const b = this.bodies[j];

                // Skip if both static/kinematic
                if (a.type !== BODY_DYNAMIC && b.type !== BODY_DYNAMIC) continue;

                const contact = detectCollision(a, b);
                if (contact) {
                    resolveContact(contact);
                }
            }
        }
    }
}

// ============================================================================
// Export
// ============================================================================

export {
    Vec3,
    SphereShape, PlaneShape, BoxShape,
    SHAPE_SPHERE, SHAPE_PLANE, SHAPE_BOX,
    BODY_STATIC, BODY_DYNAMIC, BODY_KINEMATIC,
    RigidBody,
    PhysicsWorld,
};
