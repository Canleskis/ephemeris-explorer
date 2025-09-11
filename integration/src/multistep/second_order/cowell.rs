use crate::{
    multistep::second_order::ELM2,
    problem::{Problem, SecondOrderState, State},
    ratio::Ratio,
};
use num_traits::One;
use std::ops::{Add, Div, Mul, Sub};

pub trait CowellVelocityCoefficients {
    const BETA_N: &'static [i128];

    const BETA_D: i128;
}

pub struct Cowell<const ORDER: usize>;

impl<const ORDER: usize> Cowell<ORDER> {
    #[inline]
    pub fn update_velocity<C, V, P>(lm: &mut ELM2<C, ORDER, V>, problem: &mut P, h: P::Time)
    where
        V: State,
        P: Problem<Variable = V::Variable, State = SecondOrderState<V>>,
        P::Variable: Mul<P::Time, Output = P::Variable>
            + Div<P::Time, Output = P::Variable>
            + Add<Output = P::Variable>
            + Sub<Output = P::Variable>
            + Default
            + Copy,
        P::Time: One + Mul<Ratio, Output = P::Time> + Copy,
        Self: CowellVelocityCoefficients,
    {
        let problem = problem.as_mut();

        lm.sum1.zero();
        for (j, ddy) in std::iter::once(&lm.current_ddy)
            .chain(lm.steps.iter().map(|s| &s.ddy))
            .enumerate()
        {
            for (work, ddy) in lm.sum1.iter_mut().zip(ddy.iter()) {
                *work = *work + *ddy * (P::Time::one() * Ratio::from_int(Self::BETA_N[j]));
            }
        }

        for ((dy, (y, ym1)), work) in problem
            .state
            .dy
            .iter_mut()
            .zip(problem.state.y.iter().zip(lm.steps.front().state.y.iter()))
            .zip(lm.sum1.iter())
        {
            *dy = (*y - *ym1) / h + *work * (h * Ratio::from_recip(Self::BETA_D));
        }
    }
}

impl CowellVelocityCoefficients for Cowell<1> {
    const BETA_N: &'static [i128] = &[1];

    const BETA_D: i128 = 2;
}

impl CowellVelocityCoefficients for Cowell<2> {
    const BETA_N: &'static [i128] = &[2, 1];

    const BETA_D: i128 = 6;
}

impl CowellVelocityCoefficients for Cowell<3> {
    const BETA_N: &'static [i128] = &[7, 6, -1];
    const BETA_D: i128 = 24;
}

impl CowellVelocityCoefficients for Cowell<4> {
    const BETA_N: &'static [i128] = &[97, 114, -39, 8];
    const BETA_D: i128 = 360;
}

impl CowellVelocityCoefficients for Cowell<5> {
    const BETA_N: &'static [i128] = &[367, 540, -282, 116, -21];
    const BETA_D: i128 = 1440;
}

impl CowellVelocityCoefficients for Cowell<6> {
    const BETA_N: &'static [i128] = &[2462, 4315, -3044, 1882, -682, 107];
    const BETA_D: i128 = 10080;
}

impl CowellVelocityCoefficients for Cowell<7> {
    const BETA_N: &'static [i128] = &[28549, 57750, -51453, 42484, -23109, 7254, -995];
    const BETA_D: i128 = 120960;
}

impl CowellVelocityCoefficients for Cowell<8> {
    const BETA_N: &'static [i128] = &[
        416173, 950684, -1025097, 1059430, -768805, 362112, -99359, 12062,
    ];
    const BETA_D: i128 = 1814400;
}

impl CowellVelocityCoefficients for Cowell<9> {
    const BETA_N: &'static [i128] = &[
        1624505, 4124232, -5225624, 6488192, -5888310, 3698920, -1522672, 369744, -40187,
    ];
    const BETA_D: i128 = 7257600;
}

impl CowellVelocityCoefficients for Cowell<10> {
    const BETA_N: &'static [i128] = &[
        52478684, 146269485, -213124908, 309028740, -336691836, 264441966, -145166580, 52880868,
        -11496000, 1129981,
    ];
    const BETA_D: i128 = 239500800;
}

impl CowellVelocityCoefficients for Cowell<11> {
    const BETA_N: &'static [i128] = &[
        205994615,
        624279150,
        -1028905077,
        1706529480,
        -2169992754,
        2045638356,
        -1403891730,
        681937992,
        -222389445,
        43721134,
        -3920121,
    ];
    const BETA_D: i128 = 958003200;
}

impl CowellVelocityCoefficients for Cowell<12> {
    const BETA_N: &'static [i128] = &[
        92158447389,
        301307140046,
        -554452444015,
        1035372815340,
        -1505150506950,
        1655690777412,
        -1363696062582,
        828085590240,
        -360089099415,
        106193749950,
        -19043781851,
        1569102436,
    ];
    const BETA_D: i128 = 435891456000;
}

impl CowellVelocityCoefficients for Cowell<13> {
    const BETA_N: &'static [i128] = &[
        1089142980505,
        3816786338508,
        -7759482946938,
        16111319179940,
        -26357208224085,
        33140932754040,
        -31849103413596,
        23209670507976,
        -12616471333665,
        4961170395260,
        -1334579000970,
        219929887188,
        -16758388163,
    ];
    const BETA_D: i128 = 5230697472000;
}

impl CowellVelocityCoefficients for Cowell<14> {
    const BETA_N: &'static [i128] = &[
        3222245907974,
        12037738451557,
        -26802725457012,
        61256305132546,
        -111377493654070,
        157573362429387,
        -173081395797144,
        147163097080284,
        -95999978168262,
        47189380167595,
        -16926084595636,
        4184066277762,
        -637654600522,
        45183033541,
    ];
    const BETA_D: i128 = 15692092416000;
}

impl CowellVelocityCoefficients for Cowell<15> {
    const BETA_N: &'static [i128] = &[
        12725213787853,
        50443731622830,
        -122113957635961,
        304637443761836,
        -609443588503323,
        958160677491634,
        -1184126424849705,
        1150710493076712,
        -875800754334177,
        516624748444466,
        -231637952269587,
        76348488342700,
        -17453674210001,
        2473509950766,
        -163769844043,
    ];
    const BETA_D: i128 = 62768369664000;
}
