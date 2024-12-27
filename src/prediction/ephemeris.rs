use super::{
    integration::{DormandPrince5, IntegrationState, StepError, PEFRL},
    trajectory::{
        DiscreteStates, FixedSegments, Segment, SegmentStorage, StateVector, Trajectory,
        TrajectoryData, DIV,
    },
    BuilderContext, TrajectoryBuilder,
};

use bevy::math::DVec3;
use bevy::prelude::*;
use hifitime::{Duration, Epoch};
use particular::prelude::*;
use particular::{gravity::newtonian::Acceleration, Interaction};

#[derive(Clone, Copy, Debug)]
pub struct SegmentSamples<const LEN: usize, T> {
    index: usize,
    xs: [T; LEN],
    ys: [T; LEN],
    zs: [T; LEN],
}

#[inline]
pub fn interpolate(deg: usize, ts: &[f64], xs: &[f64]) -> poly_it::Polynomial<f64, SegmentStorage> {
    poly_it::Polynomial::least_squares_fit(
        deg,
        std::iter::zip(ts.iter().copied(), xs.iter().copied()),
    )
    .unwrap()
}

impl<const LEN: usize> SegmentSamples<LEN, f64> {
    #[inline]
    pub fn with(position: DVec3) -> Self {
        Self {
            index: 1,
            xs: [position.x; LEN],
            ys: [position.y; LEN],
            zs: [position.z; LEN],
        }
    }

    #[inline]
    pub fn push(&mut self, position: DVec3) {
        assert!(self.index < LEN, "too many points for segment");
        self.xs[self.index] = position.x;
        self.ys[self.index] = position.y;
        self.zs[self.index] = position.z;
        self.index += 1;
    }

    #[inline]
    pub fn fill(&mut self, position: DVec3) {
        self.xs.fill(position.x);
        self.ys.fill(position.y);
        self.zs.fill(position.z);
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.index
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.len() == LEN
    }

    #[inline]
    fn interpolated(&self, ts: [f64; LEN], deg: usize) -> Segment {
        Segment {
            x: interpolate(deg, &ts, &self.xs),
            y: interpolate(deg, &ts, &self.ys),
            z: interpolate(deg, &ts, &self.zs),
        }
    }
}

#[derive(Clone, Copy, Mass)]
pub struct Body {
    state_vector: StateVector<DVec3>,
    mu: f64,
}

impl Position for Body {
    type Vector = DVec3;

    #[inline]
    fn position(&self) -> DVec3 {
        self.state_vector.position
    }
}

impl Body {
    #[inline]
    pub fn accelerations(_t: f64, state: &[Self], ddy: &mut Vec<DVec3>) {
        ddy.extend(state.brute_force_pairs(Acceleration::checked()));
    }
}

impl IntegrationState for Body {
    #[inline]
    fn position(&mut self) -> &mut DVec3 {
        &mut self.state_vector.position
    }

    #[inline]
    fn velocity(&mut self) -> &mut DVec3 {
        &mut self.state_vector.velocity
    }
}

#[derive(Clone, Copy, Debug)]
pub struct InterpolationBuilder<const LEN: usize> {
    time: Duration,
    degree: usize,
    samples: SegmentSamples<LEN, f64>,
}

#[derive(Component)]
pub struct FixedSegmentsBuilder<D> {
    entities: Vec<Entity>,
    integrator: PEFRL<Body>,
    interpolation: Vec<InterpolationBuilder<{ DIV + 1 }>>,
    _marker: std::marker::PhantomData<fn(D)>,
}

impl<D> Clone for FixedSegmentsBuilder<D> {
    fn clone(&self) -> Self {
        Self {
            entities: self.entities.clone(),
            integrator: self.integrator.clone(),
            interpolation: self.interpolation.clone(),
            _marker: std::marker::PhantomData,
        }
    }
}

pub trait Direction {
    fn signed_delta(&self) -> Duration;
}

impl<D: Direction> FixedSegmentsBuilder<D> {
    pub fn new(
        direction: D,
        initial_time: Epoch,
        initial_state: Vec<(Entity, DVec3, DVec3, f64, usize)>,
    ) -> Self {
        let (entities, (initial_state, interpolation_state)): (Vec<_>, (Vec<_>, Vec<_>)) =
            initial_state
                .into_iter()
                .map(|(e, v, p, m, deg)| {
                    (
                        e,
                        (
                            Body {
                                state_vector: StateVector::new(p, v),
                                mu: m,
                            },
                            InterpolationBuilder {
                                time: Duration::ZERO,
                                degree: deg,
                                samples: SegmentSamples::with(p),
                            },
                        ),
                    )
                })
                .unzip();
        Self {
            entities,
            integrator: PEFRL::new(
                initial_time.to_tai_seconds(),
                direction.signed_delta().to_seconds(),
                initial_state,
            ),
            interpolation: interpolation_state,
            _marker: std::marker::PhantomData,
        }
    }

    #[inline]
    pub fn integrator(&self) -> &PEFRL<Body> {
        &self.integrator
    }

    #[inline]
    pub fn time(&self) -> Epoch {
        Epoch::from_tai_seconds(self.integrator.time())
    }

    #[inline]
    pub fn delta(&self) -> Duration {
        Duration::from_seconds(self.integrator.delta())
    }

    #[inline]
    fn step_with<'a, I, F>(&mut self, trajectories: I, add: F) -> Result<(), StepError>
    where
        F: Fn(&mut FixedSegments, &InterpolationBuilder<{ DIV + 1 }>),
        I: IntoIterator<Item = &'a mut FixedSegments>,
    {
        self.integrator.step(Body::accelerations)?;

        let delta = self.delta();
        self.integrator
            .state()
            .iter()
            .zip(self.interpolation.iter_mut())
            .zip(trajectories)
            .for_each(|((body, interp), trajectory)| {
                interp.time += delta;
                let time = interp.time.total_nanoseconds();
                let interval = trajectory.granule().total_nanoseconds() / DIV as i128;
                if time % interval == 0 {
                    interp.samples.push(body.state_vector.position);
                    if interp.samples.is_full() {
                        add(trajectory, interp);
                        interp.time = Duration::ZERO;
                        interp.samples.fill(body.state_vector.position);
                        interp.samples.index = 1;
                    }
                }
            });

        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Forward {
    delta: Duration,
}

impl Forward {
    pub fn new(delta: Duration) -> Self {
        Self { delta }
    }
}

impl Direction for Forward {
    fn signed_delta(&self) -> Duration {
        self.delta
    }
}

impl TrajectoryBuilder for FixedSegmentsBuilder<Forward> {
    type Trajectory = FixedSegments;

    type Context = ();

    #[inline]
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering {
        lhs.cmp(rhs)
    }

    #[inline]
    fn add(to: Epoch, duration: Duration) -> Epoch {
        to + duration
    }

    #[inline]
    fn boundary(trajectory: &Self::Trajectory) -> Epoch {
        trajectory.end()
    }

    #[inline]
    fn continued(&self, trajectory: &Self::Trajectory) -> Self::Trajectory {
        // Not tested when time < end.
        let end = trajectory
            .index(self.time())
            .map(|i| trajectory.start() + trajectory.granule() * i as i64)
            .unwrap_or(trajectory.end());
        FixedSegments::new(end, trajectory.granule())
    }

    #[inline]
    fn join(lhs: &mut Self::Trajectory, rhs: Self::Trajectory) {
        lhs.clear_after(rhs.start());
        lhs.append(rhs);
    }

    #[inline]
    fn entities(&self) -> &[Entity] {
        &self.entities
    }

    #[inline]
    fn step<'a, I>(&mut self, trajs: I, _: &Self::Context) -> Result<(), StepError>
    where
        I: IntoIterator<Item = &'a mut Self::Trajectory>,
    {
        self.step_with(trajs, |trajectory, interp| {
            trajectory.push_back(interp.samples.interpolated(
                std::array::from_fn(|i| i as f64 / DIV as f64),
                interp.degree,
            ));
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Backward {
    delta: Duration,
}

impl Backward {
    pub fn new(delta: Duration) -> Self {
        Self { delta }
    }
}

impl Direction for Backward {
    fn signed_delta(&self) -> Duration {
        -self.delta
    }
}

impl TrajectoryBuilder for FixedSegmentsBuilder<Backward> {
    type Trajectory = FixedSegments;

    type Context = ();

    #[inline]
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering {
        rhs.cmp(lhs)
    }

    #[inline]
    fn add(to: Epoch, duration: Duration) -> Epoch {
        to - duration
    }

    #[inline]
    fn boundary(trajectory: &Self::Trajectory) -> Epoch {
        trajectory.start()
    }

    #[inline]
    fn continued(&self, trajectory: &Self::Trajectory) -> Self::Trajectory {
        // Not tested when time > start.
        let start = trajectory
            .index_exclusive(self.time() + trajectory.granule())
            .map(|i| trajectory.start() + trajectory.granule() * i as i64)
            .unwrap_or(trajectory.start());
        FixedSegments::new(start, trajectory.granule())
    }

    #[inline]
    fn join(lhs: &mut Self::Trajectory, rhs: Self::Trajectory) {
        lhs.clear_before(rhs.end());
        lhs.prepend(rhs);
    }

    #[inline]
    fn entities(&self) -> &[Entity] {
        &self.entities
    }

    #[inline]
    fn step<'a, I>(&mut self, trajs: I, _: &Self::Context) -> Result<(), StepError>
    where
        I: IntoIterator<Item = &'a mut Self::Trajectory>,
    {
        self.step_with(trajs, |trajectory, interp| {
            trajectory.push_front(interp.samples.interpolated(
                std::array::from_fn(|i| 1.0 - i as f64 / DIV as f64),
                interp.degree,
            ));
        })
    }
}

#[derive(Clone, Copy, Debug, Component, Deref, DerefMut)]
pub struct Mu(pub f64);

pub struct DiscreteStatesContext {
    range: std::ops::RangeInclusive<Epoch>,
    states: bevy::utils::EntityHashMap<Entity, (FixedSegments, Mu)>,
}

impl DiscreteStatesContext {
    #[inline]
    fn iter_world_trajectories(
        world: &World,
    ) -> impl Iterator<Item = (Entity, (&FixedSegments, &Mu))> {
        world
            .iter_entities()
            .filter(|e| e.contains::<Trajectory>() && e.contains::<Mu>())
            .filter_map(|e| {
                Some((
                    e.id(),
                    (
                        e.get::<Trajectory>()?.downcast_ref::<FixedSegments>(),
                        e.get::<Mu>()?,
                    ),
                ))
            })
    }

    #[inline]
    fn compute_range<'a, I>(iter: I) -> std::ops::RangeInclusive<Epoch>
    where
        I: Iterator<Item = &'a FixedSegments>,
    {
        let (start, end) = iter.fold(
            (
                Epoch::from_tai_duration(Duration::MIN),
                Epoch::from_tai_duration(Duration::MAX),
            ),
            |(start, end), traj| (start.max(traj.start()), end.min(traj.end())),
        );

        start..=end
    }

    #[inline]
    fn from_ref_world(world: &World) -> Self {
        let states: bevy::utils::EntityHashMap<_, _> = Self::iter_world_trajectories(world)
            .map(|(e, (t, m))| (e, (t.clone(), *m)))
            .collect();

        Self {
            range: Self::compute_range(states.values().map(|(t, _)| t)),
            states,
        }
    }

    #[inline]
    fn gravitational_acceleration(
        &self,
        t: Epoch,
        y: &StateVector<DVec3>,
        dy: &mut StateVector<DVec3>,
    ) {
        dy.position = y.velocity;
        dy.velocity = self.states.values().fold(DVec3::ZERO, |acc, (traj, mu)| {
            acc + Acceleration::unchecked().compute(Between(
                &y.position,
                &(
                    traj.position(t)
                        .unwrap_or_else(|| panic!("failed to evaluate position at {}", t)),
                    **mu,
                ),
            ))
        });
    }
}

impl BuilderContext for DiscreteStatesContext {
    #[inline]
    fn from_world(world: &World) -> Self {
        DiscreteStatesContext::from_ref_world(world)
    }

    #[inline]
    fn is_valid(&self, world: &World) -> bool {
        let world_range = DiscreteStatesContext::compute_range(
            Self::iter_world_trajectories(world).map(|(_, (t, _))| t),
        );

        self.range == world_range
    }
}

pub fn direction_from_states(
    current_sv: StateVector<DVec3>,
    ref_current_sv: StateVector<DVec3>,
) -> (DVec3, DVec3, DVec3) {
    let relative_sv = current_sv - ref_current_sv;

    let prograde = relative_sv.velocity.normalize_or_zero();
    let normal = relative_sv
        .position
        .cross(relative_sv.velocity)
        .normalize_or_zero();
    let radial = prograde.cross(normal).normalize_or_zero();

    (prograde, radial, normal)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ReferenceFrame {
    Frenet(Entity),
    #[default]
    Cartesian,
}

impl ReferenceFrame {
    #[inline]
    pub fn direction(
        &self,
        (current_t, current_sv): (Epoch, StateVector<DVec3>),
        ctx: &DiscreteStatesContext,
    ) -> (DVec3, DVec3, DVec3) {
        match self {
            Self::Frenet(reference) => {
                let reference_sv = ctx
                    .states
                    .get(reference)
                    .and_then(|(ref_traj, _)| ref_traj.state_vector(current_t))
                    .unwrap_or_default();

                direction_from_states(current_sv, reference_sv)
            }
            Self::Cartesian => (DVec3::X, DVec3::Y, DVec3::Z),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ConstantAcceleration {
    pub start: Epoch,
    pub duration: Duration,
    pub acceleration: DVec3,
    pub frame: ReferenceFrame,
}

impl ConstantAcceleration {
    pub fn new(
        start: Epoch,
        duration: Duration,
        acceleration: DVec3,
        frame: ReferenceFrame,
    ) -> Self {
        Self {
            start,
            duration,
            acceleration,
            frame,
        }
    }

    #[inline]
    pub fn end(&self) -> Epoch {
        self.start + self.duration
    }

    #[inline]
    fn acceleration(
        &self,
        (current_t, current_sv): (Epoch, StateVector<DVec3>),
        ctx: &DiscreteStatesContext,
    ) -> DVec3 {
        let (x_dir, y_dir, z_dir) = self.frame.direction((current_t, current_sv), ctx);
        self.acceleration.x * x_dir + self.acceleration.y * y_dir + self.acceleration.z * z_dir
    }
}

// Could be more complex eventually.
type Manoeuvre = ConstantAcceleration;

#[derive(Component, Clone)]
pub struct DiscreteStatesBuilder {
    entity: Entity,
    integrator: DormandPrince5<6, StateVector<DVec3>>,
    manoeuvres: std::collections::BTreeMap<Epoch, Option<Manoeuvre>>,
}

impl DiscreteStatesBuilder {
    pub fn new(
        entity: Entity,
        initial_time: Epoch,
        initial_velocity: DVec3,
        initial_position: DVec3,
    ) -> Self {
        Self {
            entity,
            integrator: DormandPrince5::new(
                initial_time.to_tai_seconds(),
                StateVector::new(initial_position, initial_velocity),
                1.0e-8,
                1.0e-8,
                100_000,
            ),
            manoeuvres: std::collections::BTreeMap::new(),
        }
    }

    #[inline]
    pub fn time(&self) -> Epoch {
        Epoch::from_tai_seconds(self.integrator.time())
    }

    #[expect(unused)]
    #[inline]
    pub fn state(&self) -> &StateVector<DVec3> {
        self.integrator.state()
    }

    #[inline]
    pub fn manoeuvres(&self) -> &std::collections::BTreeMap<Epoch, Option<Manoeuvre>> {
        &self.manoeuvres
    }

    #[inline]
    pub fn set_initial_state(&mut self, time: Epoch, state_vector: StateVector<DVec3>) {
        self.integrator = DormandPrince5::new(
            time.to_tai_seconds(),
            state_vector,
            self.integrator.rtol(),
            self.integrator.atol(),
            self.integrator.n_max(),
        );
    }

    #[inline]
    pub fn set_max_iterations(&mut self, max_iterations: usize) {
        self.integrator.set_n_max(max_iterations as _);
    }

    #[inline]
    pub fn insert_manoeuvre(&mut self, mut manoeuvre: Manoeuvre) {
        // self.manoeuvres
        //     .last_key_value()
        //     .inspect(|(max, _)| assert!(manoeuvre.start >= **max, "cannot overlap manoeuvres"));

        // The integrator struggles to stop at times with sub-decisecond precision, so for now we
        // simply round to the nearest 100ms as burns almost never require such precision.
        // Allowing lower tolerances during burns can fix this.
        manoeuvre.start = manoeuvre.start.round(Duration::from_milliseconds(100.0));
        manoeuvre.duration = manoeuvre.duration.round(Duration::from_milliseconds(100.0));
        self.manoeuvres.insert(manoeuvre.start, Some(manoeuvre));
        self.manoeuvres.insert(manoeuvre.end(), None);
    }

    #[inline]
    pub fn clear_manoeuvres(&mut self) {
        self.manoeuvres.clear();
    }
}

impl TrajectoryBuilder for DiscreteStatesBuilder {
    type Trajectory = DiscreteStates;

    type Context = DiscreteStatesContext;

    #[inline]
    fn cmp(lhs: &Epoch, rhs: &Epoch) -> std::cmp::Ordering {
        lhs.cmp(rhs)
    }

    #[inline]
    fn add(to: Epoch, duration: Duration) -> Epoch {
        to + duration
    }

    #[inline]
    fn boundary(trajectory: &Self::Trajectory) -> Epoch {
        trajectory.end()
    }

    #[inline]
    fn continued(&self, trajectory: &Self::Trajectory) -> Self::Trajectory {
        let sv = trajectory
            .get(self.time())
            .expect("builder was invalid when extending the prediction");
        DiscreteStates::new(self.time(), sv.velocity, sv.position)
    }

    /// Appends the new trajectory to the existing one, discarding any points that are after the
    /// start of the appended trajectory.
    #[inline]
    fn join(lhs: &mut Self::Trajectory, rhs: Self::Trajectory) {
        lhs.clear_after(rhs.start() - Duration::EPSILON);
        lhs.extend(rhs);
    }

    #[inline]
    fn entities(&self) -> &[Entity] {
        std::slice::from_ref(&self.entity)
    }

    #[inline]
    fn step<'a, I>(&mut self, trajs: I, ctx: &Self::Context) -> Result<(), StepError>
    where
        I: IntoIterator<Item = &'a mut Self::Trajectory>,
    {
        let mut next_manoeuvre_time = self.manoeuvres.range(self.time()..);
        let max_time = match next_manoeuvre_time.next().map(|(t, _)| t) {
            // If we are exactly at the start of a manoeuvre, we reset the integrator and get the end
            // time of the manoeuvre.
            Some(t) if *t == self.time() => {
                self.integrator.reset();
                next_manoeuvre_time.next().map(|(t, _)| t)
            }
            // Otherwise we have started integrating the manoeuvre and this is its end time.
            t => t,
        }
        .map_or(*ctx.range.end(), |max| max.min(*ctx.range.end()))
        .to_tai_seconds();

        self.integrator.step(max_time, 1.0, |t, y, dy| {
            let t = Epoch::from_tai_seconds(t);
            ctx.gravitational_acceleration(t, y, dy);
            if let Some((_, Some(manoeuvre))) = self.manoeuvres.range(..t).next_back() {
                dy.velocity += manoeuvre.acceleration((t, *y), ctx);
            }
        })?;

        trajs.into_iter().next().unwrap().push(
            self.time(),
            self.integrator.state().velocity,
            self.integrator.state().position,
        );

        Ok(())
    }
}
