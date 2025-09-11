#[derive(Clone, Copy, Debug)]
pub struct ODEProblem<T, V, ODE> {
    pub time: T,
    pub bound: T,
    pub state: V,
    pub ode: ODE,
}

impl<T, V, ODE> ODEProblem<T, V, ODE> {
    #[inline]
    pub fn new(time: T, bound: T, state: V, ode: ODE) -> Self {
        Self {
            time,
            bound,
            state,
            ode,
        }
    }
}

pub trait State: Sized {
    type Variable;

    type Iterator<'a>: Iterator<Item = &'a Self::Variable>
    where
        Self: 'a;

    type IteratorMut<'a>: Iterator<Item = &'a mut Self::Variable>
    where
        Self: 'a;

    fn iter(&self) -> Self::Iterator<'_>;

    fn iter_mut(&mut self) -> Self::IteratorMut<'_>;

    #[inline]
    fn fill(&mut self, value: Self::Variable) -> &mut Self
    where
        Self::Variable: Copy,
    {
        for var in self.iter_mut() {
            *var = value;
        }
        self
    }

    #[inline]
    fn filled(mut self, value: Self::Variable) -> Self
    where
        Self::Variable: Copy,
    {
        self.fill(value);
        self
    }

    #[inline]
    fn zero(&mut self) -> &mut Self
    where
        Self::Variable: Default + Copy,
    {
        self.fill(Default::default())
    }

    #[inline]
    fn zeroed(self) -> Self
    where
        Self::Variable: Default + Copy,
    {
        self.filled(Default::default())
    }
}

impl<I, Item> State for I
where
    for<'a> &'a I: IntoIterator<Item = &'a Item>,
    for<'a> &'a mut I: IntoIterator<Item = &'a mut Item>,
{
    type Variable = Item;

    type Iterator<'a>
        = <&'a I as IntoIterator>::IntoIter
    where
        Self: 'a;

    type IteratorMut<'a>
        = <&'a mut I as IntoIterator>::IntoIter
    where
        Self: 'a;

    #[inline]
    fn iter(&self) -> Self::Iterator<'_> {
        self.into_iter()
    }

    #[inline]
    fn iter_mut(&mut self) -> Self::IteratorMut<'_> {
        self.into_iter()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SecondOrderState<V> {
    pub y: V,
    pub dy: V,
}

impl<V> SecondOrderState<V> {
    #[inline]
    pub fn new(y: V, dy: V) -> Self {
        Self { y, dy }
    }
}

#[derive(Debug, Clone)]
pub struct EvalFailed;

/// Trait for first-order ordinary differential equations of the form: `y' = f(t, y)`
pub trait FirstOrderODE<T, V> {
    fn eval(&mut self, t: T, y: &V, dy: &mut V) -> Result<(), EvalFailed>;
}

/// Trait for second-order ordinary differential equations of the form: `y'' = f(t, y)`
pub trait SecondOrderODE<T, V> {
    fn eval(&mut self, t: T, y: &V, ddy: &mut V) -> Result<(), EvalFailed>;
}

/// Trait for second-order ordinary differential equations of the form: `y'' = f(t, y, y')`
pub trait SecondOrderODEGeneral<T, V> {
    fn eval(&mut self, t: T, y: &V, dy: &V, ddy: &mut V) -> Result<(), EvalFailed>;
}

impl<T, F: FnMut(T, &V, &mut V) -> Result<(), EvalFailed>, V> FirstOrderODE<T, V> for F {
    #[inline]
    fn eval(&mut self, t: T, y: &V, dy: &mut V) -> Result<(), EvalFailed> {
        self(t, y, dy)
    }
}

impl<T, F: FnMut(T, &V, &mut V) -> Result<(), EvalFailed>, V> SecondOrderODE<T, V> for F {
    #[inline]
    fn eval(&mut self, t: T, y: &V, ddy: &mut V) -> Result<(), EvalFailed> {
        self(t, y, ddy)
    }
}

impl<T, F: FnMut(T, &V, &V, &mut V) -> Result<(), EvalFailed>, V> SecondOrderODEGeneral<T, V>
    for F
{
    #[inline]
    fn eval(&mut self, t: T, y: &V, dy: &V, ddy: &mut V) -> Result<(), EvalFailed> {
        self(t, y, dy, ddy)
    }
}

pub trait Problem:
    AsRef<ODEProblem<Self::Time, Self::State, Self::ODE>>
    + AsMut<ODEProblem<Self::Time, Self::State, Self::ODE>>
{
    type Variable;

    type Time;

    type State;

    type ODE;
}
impl<P> Problem for &mut P
where
    P: Problem,
{
    type Variable = P::Variable;

    type Time = P::Time;

    type State = P::State;

    type ODE = P::ODE;
}

impl<T, V, ODE> AsRef<ODEProblem<T, V, ODE>> for ODEProblem<T, V, ODE> {
    #[inline]
    fn as_ref(&self) -> &ODEProblem<T, V, ODE> {
        self
    }
}

impl<T, V, ODE> AsMut<ODEProblem<T, V, ODE>> for ODEProblem<T, V, ODE> {
    #[inline]
    fn as_mut(&mut self) -> &mut ODEProblem<T, V, ODE> {
        self
    }
}

impl<T, V: State, ODE> Problem for ODEProblem<T, V, ODE> {
    type Variable = V::Variable;

    type Time = T;

    type State = V;

    type ODE = ODE;
}

impl<T, V: State, ODE> Problem for ODEProblem<T, SecondOrderState<V>, ODE> {
    type Variable = V::Variable;

    type Time = T;

    type State = SecondOrderState<V>;

    type ODE = ODE;
}
