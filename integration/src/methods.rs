use crate::{
    multistep::{LinearMultistep, Substepper},
    runge_kutta::{AdaptiveRungeKutta, FixedRungeKutta},
};

pub type RK4<T> = FixedRungeKutta<coeffs::RK4, T>;

pub type CashKarp45<T, Tol, U> = AdaptiveRungeKutta<coeffs::CashKarp45, T, Tol, U>;
pub type DormandPrince54<T, Tol, U> = AdaptiveRungeKutta<coeffs::DormandPrince54, T, Tol, U>;
pub type DormandPrince87<T, Tol, U> = AdaptiveRungeKutta<coeffs::DormandPrince87, T, Tol, U>;
pub type Fehlberg45<T, Tol, U> = AdaptiveRungeKutta<coeffs::Felhberg45, T, Tol, U>;
pub type Tsitouras75<T, Tol, U> = AdaptiveRungeKutta<coeffs::Tsitouras75, T, Tol, U>;
pub type Verner87<T, Tol, U> = AdaptiveRungeKutta<coeffs::Verner87, T, Tol, U>;

pub type Tsitouras75Nystrom<T, Tol, U> = AdaptiveRungeKutta<coeffs::Tsitouras75Nystrom, T, Tol, U>;

pub type Fine45<T, Tol, U> = AdaptiveRungeKutta<coeffs::Fine45, T, Tol, U>;

pub type BlanesMoan6B<T> = FixedRungeKutta<coeffs::BlanesMoan6B, T>;
pub type BlanesMoan11B<T> = FixedRungeKutta<coeffs::BlanesMoan11B, T>;
pub type BlanesMoan14A<T> = FixedRungeKutta<coeffs::BlanesMoan14A, T>;
pub type ForestRuth<T> = FixedRungeKutta<coeffs::ForestRuth, T>;
pub type McLachlanO4<T> = FixedRungeKutta<coeffs::McLachlanO4, T>;
pub type McLachlanSS17<T> = FixedRungeKutta<coeffs::McLachlanSS17, T>;
pub type Pefrl<T> = FixedRungeKutta<coeffs::Pefrl, T>;
pub type Ruth<T> = FixedRungeKutta<coeffs::Ruth, T>;

pub type AdamsBashforth2<T, S = RK4<T>> = LinearMultistep<coeffs::AdamsBashforth2, T, S>;
pub type AdamsBashforth3<T, S = RK4<T>> = LinearMultistep<coeffs::AdamsBashforth3, T, S>;
pub type AdamsBashforth4<T, S = RK4<T>> = LinearMultistep<coeffs::AdamsBashforth4, T, S>;
pub type AdamsBashforth5<T, S = Substepper<2, RK4<T>>> =
    LinearMultistep<coeffs::AdamsBashforth5, T, S>;
pub type AdamsBashforth6<T, S = Substepper<2, RK4<T>>> =
    LinearMultistep<coeffs::AdamsBashforth6, T, S>;

pub type QuinlanTremaine12<T, S = Substepper<4, BlanesMoan6B<T>>> =
    LinearMultistep<coeffs::QuinlanTremaine12, T, S>;
pub type Stormer13<T, S = Substepper<4, BlanesMoan6B<T>>> =
    LinearMultistep<coeffs::Stormer13, T, S>;

pub mod coeffs {
    use crate::{
        multistep::{ELM1, ELM1Coefficients, ELM2, ELM2Coefficients, LMCoefficients},
        problem::{Problem, SecondOrderState},
        ratio::Ratio,
        runge_kutta::{
            EERKCoefficients, EERKNCoefficients, EERKNGCoefficients, ERK, ERKCoefficients, ERKN,
            ERKNCoefficients, ERKNG, ERKNGCoefficients, RKCoefficients, SRKN, SRKNCoefficients,
        },
    };

    #[macro_export]
    macro_rules! frac {
        ($numer:expr, $denom:expr) => {
            Ratio::const_new($numer, $denom)
        };
    }

    #[macro_export]
    macro_rules! frac_f64 {
        ($frac:expr) => {
            Ratio::from_f64($frac).expect("failed to convert f64 to Ratio")
        };
    }

    #[doc(hidden)]
    #[derive(Debug, Clone, Copy)]
    pub struct RK4;
    impl<P: Problem> RKCoefficients<P> for RK4 {
        type Instance = ERK<Self, [P::State; 4]>;
    }
    impl ERKCoefficients for RK4 {
        const FSAL: bool = false;

        const ORDER: u16 = 4;

        const A: &'static [&'static [Ratio]] = &[
            &[],
            &[frac!(1, 2)],
            &[frac!(0, 1), frac!(1, 2)],
            &[frac!(0, 1), frac!(0, 1), frac!(1, 1)],
        ];

        const B: &'static [Ratio] = &[frac!(1, 6), frac!(1, 3), frac!(1, 3), frac!(1, 6)];

        const C: &'static [Ratio] = &[frac!(0, 1), frac!(1, 2), frac!(1, 2), frac!(1, 1)];
    }

    #[doc(hidden)]
    #[derive(Debug, Clone, Copy)]
    pub struct CashKarp45;
    impl<P: Problem> RKCoefficients<P> for CashKarp45 {
        type Instance = ERK<Self, [P::State; 6]>;
    }
    impl ERKCoefficients for CashKarp45 {
        const FSAL: bool = false;

        const ORDER: u16 = 4;

        const A: &'static [&'static [Ratio]] = &[
            &[],
            &[frac!(1, 5)],
            &[frac!(3, 40), frac!(9, 40)],
            &[frac!(3, 10), frac!(-9, 10), frac!(6, 5)],
            &[frac!(-11, 54), frac!(5, 2), frac!(-70, 27), frac!(35, 27)],
            &[
                frac!(1631, 55296),
                frac!(175, 512),
                frac!(575, 13824),
                frac!(44275, 110592),
                frac!(253, 4096),
            ],
        ];

        const B: &'static [Ratio] = &[
            frac!(2825, 27648),
            frac!(0, 1),
            frac!(18575, 48384),
            frac!(13525, 55296),
            frac!(277, 14336),
            frac!(1, 4),
        ];

        const C: &'static [Ratio] = &[
            frac!(0, 1),
            frac!(1, 5),
            frac!(3, 10),
            frac!(3, 5),
            frac!(1, 1),
            frac!(7, 8),
        ];
    }
    impl EERKCoefficients for CashKarp45 {
        const ORDER_EMBEDDED: u16 = 5;

        const E: &'static [Ratio] = {
            const BH: &[Ratio] = &[
                frac!(37, 378),
                frac!(0, 1),
                frac!(250, 621),
                frac!(125, 594),
                frac!(0, 1),
                frac!(512, 1771),
            ];

            &[
                Self::B[0].const_sub(BH[0]),
                Self::B[1].const_sub(BH[1]),
                Self::B[2].const_sub(BH[2]),
                Self::B[3].const_sub(BH[3]),
                Self::B[4].const_sub(BH[4]),
                Self::B[5].const_sub(BH[5]),
            ]
        };
    }

    #[doc(hidden)]
    #[derive(Debug, Clone, Copy)]
    pub struct DormandPrince54;
    impl<P: Problem> RKCoefficients<P> for DormandPrince54 {
        type Instance = ERK<Self, [P::State; 7]>;
    }
    impl ERKCoefficients for DormandPrince54 {
        const FSAL: bool = true;

        const ORDER: u16 = 5;

        const A: &'static [&'static [Ratio]] = &[
            &[],
            &[frac!(1, 5)],
            &[frac!(3, 40), frac!(9, 40)],
            &[frac!(44, 45), frac!(-56, 15), frac!(32, 9)],
            &[
                frac!(19372, 6561),
                frac!(-25360, 2187),
                frac!(64448, 6561),
                frac!(-212, 729),
            ],
            &[
                frac!(9017, 3168),
                frac!(-355, 33),
                frac!(46732, 5247),
                frac!(49, 176),
                frac!(-5103, 18656),
            ],
            &[
                frac!(35, 384),
                frac!(0, 1),
                frac!(500, 1113),
                frac!(125, 192),
                frac!(-2187, 6784),
                frac!(11, 84),
            ],
        ];

        const B: &'static [Ratio] = &[
            Self::A[6][0],
            Self::A[6][1],
            Self::A[6][2],
            Self::A[6][3],
            Self::A[6][4],
            Self::A[6][5],
            frac!(0, 1),
        ];

        const C: &'static [Ratio] = &[
            frac!(0, 1),
            frac!(1, 5),
            frac!(3, 10),
            frac!(4, 5),
            frac!(8, 9),
            frac!(1, 1),
            frac!(1, 1),
        ];
    }
    impl EERKCoefficients for DormandPrince54 {
        const ORDER_EMBEDDED: u16 = 4;

        const E: &'static [Ratio] = {
            const BH: &[Ratio] = &[
                frac!(5179, 57600),
                frac!(0, 1),
                frac!(7571, 16695),
                frac!(393, 640),
                frac!(-92097, 339200),
                frac!(187, 2100),
                frac!(1, 40),
            ];

            &[
                Self::B[0].const_sub(BH[0]),
                Self::B[1].const_sub(BH[1]),
                Self::B[2].const_sub(BH[2]),
                Self::B[3].const_sub(BH[3]),
                Self::B[4].const_sub(BH[4]),
                Self::B[5].const_sub(BH[5]),
                Self::B[6].const_sub(BH[6]),
            ]
        };
    }

    #[doc(hidden)]
    #[derive(Debug, Clone, Copy)]
    pub struct DormandPrince87;
    impl<P: Problem> RKCoefficients<P> for DormandPrince87 {
        type Instance = ERK<Self, [P::State; 13]>;
    }
    impl ERKCoefficients for DormandPrince87 {
        const FSAL: bool = false;

        const ORDER: u16 = 8;

        const A: &'static [&'static [Ratio]] = &[
            &[],
            &[frac!(1, 18)],
            &[frac!(1, 48), frac!(1, 16)],
            &[frac!(1, 32), frac!(0, 1), frac!(3, 32)],
            &[frac!(5, 16), frac!(0, 1), frac!(-75, 64), frac!(75, 64)],
            &[
                frac!(3, 80),
                frac!(0, 1),
                frac!(0, 1),
                frac!(3, 16),
                frac!(3, 20),
            ],
            &[
                frac!(29443841, 614563906),
                frac!(0, 1),
                frac!(0, 1),
                frac!(77736538, 692538347),
                frac!(-28693883, 1125000000),
                frac!(23124283, 1800000000),
            ],
            &[
                frac!(16016141, 946692911),
                frac!(0, 1),
                frac!(0, 1),
                frac!(61564180, 158732637),
                frac!(22789713, 633445777),
                frac!(545815736, 2771057229),
                frac!(-180193667, 1043307555),
            ],
            &[
                frac!(39632708, 573591083),
                frac!(0, 1),
                frac!(0, 1),
                frac!(-433636366, 683701615),
                frac!(-421739975, 2616292301),
                frac!(100302831, 723423059),
                frac!(790204164, 839813087),
                frac!(800635310, 3783071287),
            ],
            &[
                frac!(246121993, 1340847787),
                frac!(0, 1),
                frac!(0, 1),
                frac!(-37695042795, 15268766246),
                frac!(-309121744, 1061227803),
                frac!(-12992083, 490766935),
                frac!(6005943493, 2108947869),
                frac!(393006217, 1396673457),
                frac!(123872331, 1001029789),
            ],
            &[
                frac!(-1028468189, 846180014),
                frac!(0, 1),
                frac!(0, 1),
                frac!(8478235783, 508512852),
                frac!(1311729495, 1432422823),
                frac!(-10304129995, 1701304382),
                frac!(-48777925059, 3047939560),
                frac!(15336726248, 1032824649),
                frac!(-45442868181, 3398467696),
                frac!(3065993473, 597172653),
            ],
            &[
                frac!(185892177, 718116043),
                frac!(0, 1),
                frac!(0, 1),
                frac!(-3185094517, 667107341),
                frac!(-477755414, 1098053517),
                frac!(-703635378, 230739211),
                frac!(5731566787, 1027545527),
                frac!(5232866602, 850066563),
                frac!(-4093664535, 808688257),
                frac!(3962137247, 1805957418),
                frac!(65686358, 487910083),
            ],
            &[
                frac!(403863854, 491063109),
                frac!(0, 1),
                frac!(0, 1),
                frac!(-5068492393, 434740067),
                frac!(-411421997, 543043805),
                frac!(652783627, 914296604),
                frac!(11173962825, 925320556),
                frac!(-13158990841, 6184727034),
                frac!(3936647629, 1978049680),
                frac!(-160528059, 685178525),
                frac!(248638103, 1413531060),
                frac!(0, 1),
            ],
        ];

        const B: &'static [Ratio] = &[
            frac!(14005451, 335480064),
            frac!(0, 1),
            frac!(0, 1),
            frac!(0, 1),
            frac!(0, 1),
            frac!(-59238493, 1068277825),
            frac!(181606767, 758867731),
            frac!(561292985, 797845732),
            frac!(-1041891430, 1371343529),
            frac!(760417239, 1151165299),
            frac!(118820643, 751138087),
            frac!(-528747749, 2220607170),
            frac!(1, 4),
        ];

        const C: &'static [Ratio] = &[
            frac!(0, 1),
            frac!(1, 18),
            frac!(1, 12),
            frac!(1, 8),
            frac!(5, 16),
            frac!(3, 8),
            frac!(59, 400),
            frac!(93, 200),
            frac!(5490023248, 9719169821),
            frac!(13, 20),
            frac!(1201146811, 1299019798),
            frac!(1, 1),
            frac!(1, 1),
        ];
    }
    impl EERKCoefficients for DormandPrince87 {
        const ORDER_EMBEDDED: u16 = 7;

        const E: &'static [Ratio] = {
            const BH: &[Ratio] = &[
                frac!(13451932, 455176623),
                frac!(0, 1),
                frac!(0, 1),
                frac!(0, 1),
                frac!(0, 1),
                frac!(-808719846, 976000145),
                frac!(1757004468, 5645159321),
                frac!(656045339, 265891186),
                frac!(-3867574721, 1518517206),
                frac!(465885868, 322736535),
                frac!(53011238, 667516719),
                frac!(2, 45),
                frac!(0, 1),
            ];

            &[
                Self::B[0].const_sub(BH[0]),
                Self::B[1].const_sub(BH[1]),
                Self::B[2].const_sub(BH[2]),
                Self::B[3].const_sub(BH[3]),
                Self::B[4].const_sub(BH[4]),
                Self::B[5].const_sub(BH[5]),
                Self::B[6].const_sub(BH[6]),
                Self::B[7].const_sub(BH[7]),
                Self::B[8].const_sub(BH[8]),
                Self::B[9].const_sub(BH[9]),
                Self::B[10].const_sub(BH[10]),
                Self::B[11].const_sub(BH[11]),
                Self::B[12].const_sub(BH[12]),
            ]
        };
    }

    #[doc(hidden)]
    #[derive(Debug, Clone, Copy)]
    pub struct Felhberg45;
    impl<P: Problem> RKCoefficients<P> for Felhberg45 {
        type Instance = ERK<Self, [P::State; 6]>;
    }
    impl ERKCoefficients for Felhberg45 {
        const FSAL: bool = false;

        const ORDER: u16 = 4;

        const A: &'static [&'static [Ratio]] = &[
            &[],
            &[frac!(1, 4)],
            &[frac!(3, 32), frac!(9, 32)],
            &[frac!(1932, 2197), frac!(-7200, 2197), frac!(7296, 2197)],
            &[
                frac!(439, 216),
                frac!(-8, 1),
                frac!(3680, 513),
                frac!(-845, 4104),
            ],
            &[
                frac!(-8, 27),
                frac!(2, 1),
                frac!(-3544, 2565),
                frac!(1859, 4104),
                frac!(-11, 40),
            ],
        ];

        // Integration is continued with the 4th order solution.
        const B: &'static [Ratio] = &[
            frac!(25, 216),
            frac!(0, 1),
            frac!(1408, 2565),
            frac!(2197, 4104),
            frac!(-1, 5),
            frac!(0, 1),
        ];

        const C: &'static [Ratio] = &[
            frac!(0, 1),
            frac!(1, 4),
            frac!(3, 8),
            frac!(12, 13),
            frac!(1, 1),
            frac!(1, 2),
        ];
    }
    impl EERKCoefficients for Felhberg45 {
        const ORDER_EMBEDDED: u16 = 5;

        const E: &'static [Ratio] = {
            const BH: &[Ratio] = &[
                frac!(16, 135),
                frac!(0, 1),
                frac!(6656, 12825),
                frac!(28561, 56430),
                frac!(-9, 50),
                frac!(2, 55),
            ];

            &[
                BH[0].const_sub(Self::B[0]),
                BH[1].const_sub(Self::B[1]),
                BH[2].const_sub(Self::B[2]),
                BH[3].const_sub(Self::B[3]),
                BH[4].const_sub(Self::B[4]),
                BH[5].const_sub(Self::B[5]),
            ]
        };
    }

    #[doc(hidden)]
    #[derive(Debug, Clone, Copy)]
    pub struct Verner87;
    impl<P: Problem> RKCoefficients<P> for Verner87 {
        type Instance = ERK<Self, [P::State; 13]>;
    }
    // Some of the coefficients were adjusted here since the original numerators and denominators
    // did not fit in i128. Error is in the order of 1e-75.
    impl ERKCoefficients for Verner87 {
        const FSAL: bool = false;

        const ORDER: u16 = 8;

        const A: &'static [&'static [Ratio]] = &[
            &[],
            &[frac!(1, 4)],
            &[
                frac!(25374388559, 290322000000),
                frac!(7399612441, 290322000000),
            ],
            &[frac!(86021, 2032000), frac!(0, 1), frac!(258063, 2032000)],
            &[
                frac!(394117287349, 924951555125),
                frac!(0, 1),
                frac!(-1478808184872, 924951555125),
                frac!(1476870356896, 924951555125),
            ],
            &[
                frac!(21806069, 429936000),
                frac!(0, 1),
                frac!(0, 1),
                frac!(2085253894, 8198887125),
                frac!(67122964561, 329119824000),
            ],
            &[
                frac!(-10767669849309, 37129416272000),
                frac!(0, 1),
                frac!(0, 1),
                frac!(107549272780798434, 80010624633743375),
                frac!(-1571425354197, 548533040000),
                frac!(195974150961, 73190382500),
            ],
            &[
                frac!(122914197, 1247416480),
                frac!(0, 1),
                frac!(0, 1),
                frac!(0, 1),
                frac!(2834577, 12772576),
                frac!(-1123911, 6195548),
                frac!(1003245, 91667332),
            ],
            &[
                frac!(
                    19132364217086473439451957038886964845,
                    49423518501555938547094094056669611451
                ), // From 7516815761873698650240937021297888610305781198156454914018066425686713809124 / 19417750920086037142705440148991358353524951455336635926349428617092008016799
                frac!(0, 1),
                frac!(0, 1),
                frac!(
                    -143051247528307045267782109157359240437,
                    99172722836096132636841104437743560921
                ), // From 2970280712345026029784493078033500603558797778620788610891609609625083520000000 / 2059197881322255811366824379732129118957063276855086357832879720044283034195609
                frac!(
                    115818060834107557053794129051206227162,
                    39863059483485862013028140486896513061
                ), // From 160883051099229570661648871967605637311447402857798976770144318499344176200 / 55373838844007112802284696674027443630749586849658020349496486226959885797
                frac!(
                    -141807645667319562752386821251651947021,
                    76496849039517751054252482657799700737
                ), // From 531291044569132007849980767445890096567679581535833821522948115776782916800 / 286600138103969709516639731800408929385020510549016787456022559956334619913
                frac!(
                    16888569889988038039976943865754544366,
                    120601215989724130931828824618075858583
                ), // From 92917760295507691123384296375592154091398238056386167422408760067912000 / 663525387387781548392930054085313627947883220329590736767284727992793459
                frac!(
                    52398059241656717894088436624694164617,
                    91486736374773028520690730890654808410
                ), // From 16582882008248100072444682934459438760351720266822956376058566080 / 28953624935338209620540938361130162616618239541632595427443431123
            ],
            &[
                frac!(
                    -11547721995303103469972658432503594540,
                    71616429315315057150392830005082242033
                ), // From -397907419247815925921130059686865048509124 / 2467734205602744466122026313943800290709375
                frac!(0, 1),
                frac!(0, 1),
                frac!(
                    -12397380504027867540933244272640,
                    71497487771289358131961402817451
                ),
                frac!(
                    -938215172932764769802614036208,
                    720988934849815411624418693625
                ),
                frac!(
                    10929371706048739692132704216736592273,
                    9604436137541494989502494523975339525
                ), // From 39294182008628611505961825730565883738506112 / 34530663960302355453963385168366837619171375
                frac!(
                    -4957840628781806512193170061009797616,
                    156164020998659418993865796134637580607
                ), // From -44522287344548287709730039702917122688 / 1402380579847472465941191457456761953225
                frac!(
                    20222100246339043804092120397184,
                    21662367405707901792558292907875
                ),
                frac!(
                    -14194298842270754526573828795828789935,
                    169410700032948794997551526748733133541
                ), // From -30521958981544768864151899144647092370777642374348985368652529884 / 364283329166067503011761408054691133551081505869170977618646228125
            ],
            &[
                frac!(
                    -6595977530126058092276113463162447,
                    343550429234075503704418976591029925
                ), // From -7874248153103427073301520825686819245245713313 / 410128949126757953376487799591079982676499635200
                frac!(0, 1),
                frac!(0, 1),
                frac!(
                    592147767606693369421771800000,
                    2166590538523919943392769782347
                ),
                frac!(-3777316336297553135210313111, 5593126282471295314419732896),
                frac!(
                    46584458703283890079268290864635636608,
                    136403910643801738344046281800231160435
                ), // From 160775746506894169042900490691204481389877265439 / 470767315337957626238784466576517627715500536844
                frac!(
                    -3305130278623094384653285667606996107,
                    48640575813827551603190225789521663509
                ), // From -507593940096706904938228151863398894238905405 / 7470102369519587845109530946292707809479335476
                frac!(
                    173196146788876378135182681744256899,
                    1793073867682496036448646518502714730
                ),
                frac!(
                    21878388731882640277597903637628571314,
                    165081510006619658468100289435710175913
                ), // From 937132687483187059957479559988037034595389945431346986697458481846645979573026347 / 7071054501415007346897024777503547537721817297910549006214731593062974677881100800
                frac!(116416656136756811965545, 315877857843708025406464),
            ],
            &[
                frac!(
                    86283455828276580969265299615567006809,
                    141636888123595247751495010447511367118
                ), // From 28191405932827173126309408200664465069287507375903 / 46277040828099807659589313374497236005056822803488
                frac!(0, 1),
                frac!(0, 1),
                frac!(
                    -151950398206566076471837140958420393571,
                    66862829011212719149302112982245013711
                ), // From -429380072388144504981858700675520000000 / 188940382517997566021881003968356031669
                frac!(
                    24173887609985281514926963208103700,
                    5080791111711202068770238453613777
                ),
                frac!(
                    -143317464005479858560111680138126832189,
                    25981633718504948264182812470902428983
                ), // From -54881449674479480016593038012420742831966782264181600 / 9949308922521596315581680829335134273237927944004721
                frac!(
                    42209916225391952608407915959962152668,
                    145521509530286370954454926879941648963
                ), // From 970969892075126686370480622782649944320946827348000 / 3347483649309697775724736894621937483768621235854151
                frac!(
                    77411823900067580361972977027037723040,
                    136014860952969119034946902128158999209
                ),
                frac!(
                    90243247413252527696921776905977090329,
                    113845808750181479776767329854872080044
                ), // From 656779806953299263849114640553093566236946159841995197013788762083697980273440465097410146645645 / 828556489672666107963907105193770544832899849879697116662334249174938654251472014083110713914032
                frac!(
                    3147129425650261440272289698280046875,
                    20338543888980253307598069078752387872
                ),
                frac!(
                    425854850025102447819650719213220000,
                    263691965699784622774911587349396263
                ),
            ],
            &[
                frac!(
                    144193174842911965173495689478384248098,
                    162497251676329524199993178672872958679
                ), // From 3621456146894606285894995339383850874108700148373 / 4081168693163363596994878822807031005774876048608
                frac!(0, 1),
                frac!(0, 1),
                frac!(
                    -62960135151985975889851790525142771812,
                    21159800421624169611319046296185651865
                ), // From -3817583686059663366913679467944640000000 / 1283023117642128560298752783599596583383
                frac!(
                    193234496869635405437524415902310900,
                    34501742641572006835386893540142539
                ),
                frac!(
                    -62972246599794059248830224223118607821,
                    10645102320655354611819459561250886696
                ), // From -87974988257716022018037206149145819377007277738400 / 14871674463411999056342756887683215806741009406229
                frac!(
                    8863347671425893103351776801794459207,
                    40233648367015601487992774127088854831
                ), // From 3275133743198491987936602396440511473300401684000 / 14866908561378488620343856456061248578257652060133
                frac!(
                    8526827543558748199980436728262708480,
                    83965981332240915696885276969279079833
                ),
                frac!(
                    57525336940606229238770624056483638657,
                    49959710001988187760933546130944592210
                ), // From 39009210105998257555064429930449313140180415214883261536546755024187088643232941505 / 33878790250537583879968593309051059917530985063051045330469429477091985229650821808
                frac!(
                    266248736648182734031056196146328125,
                    137973433143769650633804707559140704
                ),
                frac!(0, 1),
                frac!(0, 1),
            ],
        ];

        const B: &'static [Ratio] = &[
            frac!(105711409411029096011, 2363345366733216286848),
            frac!(0, 1),
            frac!(0, 1),
            frac!(0, 1),
            frac!(0, 1),
            frac!(
                49144903996079337500000000000,
                313203740909075359159534015851
            ),
            frac!(
                122908398183230837500000000000,
                665774200882383246774037878723
            ),
            frac!(14734515405327276428000, 65439093723444914604579),
            frac!(
                21775072723843685338472933144880544910,
                147182415792895885939078846146121452411
            ), // From 98855236015768950806532226039371426875067857784942240050406497922541356709067743917330653 / 668183874060994347945818441723205177438829897987551586564757651033104640132059340614221120
            frac!(80375815959623894921875, 1056804190408540040777856),
            frac!(48020889007901260000, 391135894718289455007),
            frac!(1146762646211206909, 27426666521711241540),
            frac!(0, 1),
        ];

        const C: &'static [Ratio] = &[
            frac!(0, 1),
            frac!(1, 4),
            frac!(86021, 762000),
            frac!(86021, 508000),
            frac!(53, 125),
            frac!(509, 1000),
            frac!(867, 1000),
            frac!(3, 20),
            frac!(295159845764, 416264491649),
            frac!(8, 25),
            frac!(9, 20),
            frac!(1, 1),
            frac!(1, 1),
        ];
    }
    impl EERKCoefficients for Verner87 {
        const ORDER_EMBEDDED: u16 = 7;

        const E: &'static [Ratio] = {
            const BH: &[Ratio] = &[
                frac!(415143901557332051, 9054963090931863168),
                frac!(0, 1),
                frac!(0, 1),
                frac!(0, 1),
                frac!(0, 1),
                frac!(1392529918457821250000000000, 5308537981509751850161593489),
                frac!(2142381651020498750000000000, 11176065722246241552561786933),
                frac!(4735428542080120856000, 21813031241148304868193),
                frac!(
                    3763409342279116279041709748276428031,
                    29544303022010056212357478385959560188
                ), // From 1360794378607602525113722425939330174725374220595898325328629456869721330363 / 10682792599936408020924171586589409694572260337238697644239411829708789571520
                frac!(9357212880829926171875, 81292630031426156982912),
                frac!(0, 1),
                frac!(0, 1),
                frac!(7787234078438942063, 191986665651978690780),
            ];

            &[
                frac!(-660287223858642325, 590836341683304071712), // From Self::B[0].const_sub(BH[0])
                Self::B[1].const_sub(BH[1]),
                Self::B[2].const_sub(BH[2]),
                Self::B[3].const_sub(BH[3]),
                Self::B[4].const_sub(BH[4]),
                frac!(
                    -33014361192932116250000000000,
                    313203740909075359159534015851
                ), // From Self::B[5].const_sub(BH[5])
                frac!(
                    -33014361192932116250000000000,
                    4660419406176682727418265151061
                ), // From Self::B[6].const_sub(BH[6])
                frac!(528229779086913860000, 65439093723444914604579), // From Self::B[7].const_sub(BH[7])
                frac!(
                    2761869401431568978679427864860062867,
                    134304339907491654060551956661891639298
                ), // From Self::B[8].const_sub(BH[8]) with original coefficients
                frac!(-10316987872791286328125, 264201047602135010194464), // From Self::B[9].const_sub(BH[9])
                Self::B[10].const_sub(BH[10]),
                Self::B[11].const_sub(BH[11]),
                Self::B[12].const_sub(BH[12]),
            ]
        };
    }

    #[doc(hidden)]
    #[derive(Debug, Clone, Copy)]
    pub struct Tsitouras75;
    impl<P: Problem> RKCoefficients<P> for Tsitouras75 {
        type Instance = ERK<Self, [P::State; 9]>;
    }
    impl ERKCoefficients for Tsitouras75 {
        const FSAL: bool = false;

        const ORDER: u16 = 7;

        const A: &'static [&'static [Ratio]] = &[
            &[],
            &[frac!(1, 18)],
            &[frac!(0, 1), frac!(1, 9)],
            &[frac!(1, 24), frac!(0, 1), frac!(1, 8)],
            &[
                frac!(2183971, 4000000),
                frac!(0, 1),
                frac!(-8340813, 4000000),
                frac!(3968421, 2000000),
            ],
            &[
                frac!(695768212, 7463744411),
                frac!(0, 1),
                frac!(-1803549175, 7007942496),
                frac!(3474507053, 6790877290),
                frac!(2188198899, 15264927763),
            ],
            &[
                frac!(-11894934857, 8390623634),
                frac!(0, 1),
                frac!(53094780276, 9800512003),
                frac!(-8415376229, 2277049503),
                frac!(-18647567697, 10138317907),
                frac!(27551494893, 11905950217),
            ],
            &[
                frac!(30828057951, 7654644085),
                frac!(0, 1),
                frac!(-4511704, 324729),
                frac!(16217851618, 1651177175),
                frac!(282768186839, 40694064384),
                frac!(-104400780537, 15869257619),
                frac!(5409241639, 9600177208),
            ],
            &[
                frac!(-133775720546, 36753383835),
                frac!(0, 1),
                frac!(49608695511, 4066590848),
                frac!(-59896475201, 7901259813),
                frac!(-48035527651, 5727379426),
                frac!(86266718551, 10188951048),
                frac!(-7751618114, 23575802495),
                frac!(2289274942, 8464405725),
            ],
        ];

        const B: &'static [Ratio] = &[
            frac!(597988726, 12374436915),
            frac!(0, 1),
            frac!(0, 1),
            frac!(3138312158, 11968408119),
            frac!(480882843, 7850665645),
            frac!(988558885, 3512253271),
            frac!(5302636961, 26425940286),
            frac!(1259489433, 12163586030),
            frac!(1016647712, 23899101975),
        ];

        const C: &'static [Ratio] = &[
            frac!(0, 1),
            frac!(1, 18),
            frac!(1, 9),
            frac!(1, 6),
            frac!(89, 200),
            frac!(56482, 115069),
            frac!(74, 95),
            frac!(8, 9),
            frac!(1, 1),
        ];
    }
    impl EERKCoefficients for Tsitouras75 {
        const ORDER_EMBEDDED: u16 = 5;

        const E: &'static [Ratio] = {
            const BH: &[Ratio] = &[
                frac!(1421940313, 46193547077),
                frac!(0, 1),
                frac!(0, 1),
                frac!(1943068601, 5911217046),
                frac!(-3019049881, 6506827856),
                frac!(7688913279, 9493187186),
                frac!(586186883, 5187186385),
                frac!(1114095023, 8014791121),
                frac!(1016647712, 23899101975),
            ];

            &[
                Self::B[0].const_sub(BH[0]),
                Self::B[1].const_sub(BH[1]),
                Self::B[2].const_sub(BH[2]),
                Self::B[3].const_sub(BH[3]),
                Self::B[4].const_sub(BH[4]),
                Self::B[5].const_sub(BH[5]),
                Self::B[6].const_sub(BH[6]),
                Self::B[7].const_sub(BH[7]),
                Self::B[8].const_sub(BH[8]),
            ]
        };
    }

    #[doc(hidden)]
    #[derive(Debug, Clone, Copy)]
    pub struct Tsitouras75Nystrom;
    impl<V, P: Problem<State = SecondOrderState<V>>> RKCoefficients<P> for Tsitouras75Nystrom {
        type Instance = ERKN<Self, [V; 7], V>;
    }
    impl ERKNCoefficients for Tsitouras75Nystrom {
        const FSAL: bool = true;

        const ORDER: u16 = 7;

        const A: &'static [&'static [Ratio]] = &[
            &[],
            &[frac!(5107771, 767472028)],
            &[frac!(5107771, 575604021), frac!(16661485, 938806552)],
            &[
                frac!(325996677, 876867260),
                frac!(-397622579, 499461366),
                frac!(541212017, 762248206),
            ],
            &[
                frac!(82243160, 364375691),
                frac!(-515873404, 1213273815),
                frac!(820109726, 1294837243),
                frac!(36245507, 242779260),
            ],
            &[
                frac!(3579594, 351273191),
                frac!(34292133, 461028419),
                frac!(267156948, 2671391749),
                frac!(22665163, 1338599875),
                frac!(-3836509, 1614789462),
            ],
            &[
                frac!(53103334, 780726093),
                frac!(0, 1),
                frac!(352190060, 1283966121),
                frac!(37088117, 2206150964),
                frac!(7183323, 1828127386),
                frac!(187705681, 1370684829),
            ],
        ];

        const BP: &'static [Ratio] = &[
            <Self as ERKNCoefficients>::A[6][0],
            <Self as ERKNCoefficients>::A[6][1],
            <Self as ERKNCoefficients>::A[6][2],
            <Self as ERKNCoefficients>::A[6][3],
            <Self as ERKNCoefficients>::A[6][4],
            <Self as ERKNCoefficients>::A[6][5],
            frac!(0, 1),
        ];

        const BV: &'static [Ratio] = &[
            frac!(53103334, 780726093),
            frac!(0, 1),
            frac!(244481296, 685635505),
            frac!(41493456, 602487871),
            frac!(-45498718, 926142189),
            frac!(1625563237, 4379140271),
            frac!(191595797, 1038702495),
        ];

        const C: &'static [Ratio] = &[
            frac!(0, 1),
            frac!(108816483, 943181462),
            frac!(108816483, 471590731),
            frac!(151401202, 200292705),
            frac!(682035803, 631524599),
            frac!(493263404, 781610081),
            frac!(1, 1),
        ];
    }
    impl EERKNCoefficients for Tsitouras75Nystrom {
        const ORDER_EMBEDDED: u16 = 5;

        const EP: &'static [Ratio] = {
            const BHP: &[Ratio] = &[
                frac!(41808761, 935030896),
                frac!(0, 1),
                frac!(46261019, 135447428),
                frac!(289298425, 1527932372),
                frac!(-52260067, 3104571287),
                frac!(-49872919, 848719175),
                frac!(0, 1),
            ];

            &[
                <Self as ERKNCoefficients>::BP[0].const_sub(BHP[0]),
                <Self as ERKNCoefficients>::BP[1].const_sub(BHP[1]),
                <Self as ERKNCoefficients>::BP[2].const_sub(BHP[2]),
                <Self as ERKNCoefficients>::BP[3].const_sub(BHP[3]),
                <Self as ERKNCoefficients>::BP[4].const_sub(BHP[4]),
                <Self as ERKNCoefficients>::BP[5].const_sub(BHP[5]),
                <Self as ERKNCoefficients>::BP[6].const_sub(BHP[6]),
            ]
        };

        const EV: &'static [Ratio] = {
            const BHV: &[Ratio] = &[
                frac!(41808761, 935030896),
                frac!(0, 1),
                frac!(224724272, 506147085),
                frac!(2995752066, 3862177123),
                frac!(170795979, 811534085),
                frac!(-177906423, 1116903503),
                frac!(-655510901, 2077404990),
            ];

            &[
                <Self as ERKNCoefficients>::BV[0].const_sub(BHV[0]),
                <Self as ERKNCoefficients>::BV[1].const_sub(BHV[1]),
                <Self as ERKNCoefficients>::BV[2].const_sub(BHV[2]),
                <Self as ERKNCoefficients>::BV[3].const_sub(BHV[3]),
                <Self as ERKNCoefficients>::BV[4].const_sub(BHV[4]),
                <Self as ERKNCoefficients>::BV[5].const_sub(BHV[5]),
                <Self as ERKNCoefficients>::BV[6].const_sub(BHV[6]),
            ]
        };
    }

    // Fine, J.M. Low order practical Runge-Kutta-Nyström methods. Computing 38, 281–297 (1987). https://link.springer.com/article/10.1007/BF02278707
    #[doc(hidden)]
    #[derive(Debug, Clone, Copy)]
    pub struct Fine45;
    impl<V, P: Problem<State = SecondOrderState<V>>> RKCoefficients<P> for Fine45 {
        type Instance = ERKNG<Self, [V; 7], V>;
    }
    impl ERKNGCoefficients for Fine45 {
        const FSAL: bool = true;

        const ORDER: u16 = 4;

        const AP: &'static [&'static [Ratio]] = &[
            &[],
            &[frac!(32, 1521)],
            &[frac!(4, 169), frac!(4, 169)],
            &[frac!(175, 5184), frac!(0, 1), frac!(1625, 5184)],
            &[
                frac!(-342497279, 5618900760),
                frac!(6827067, 46824173),
                frac!(35048741, 102161832),
                frac!(-2201514, 234120865),
            ],
            &[
                frac!(-7079, 52152),
                frac!(767, 2173),
                frac!(14027, 52152),
                frac!(30, 2173),
                frac!(0, 1),
            ],
            &[
                frac!(4817, 51600),
                frac!(0, 1),
                frac!(388869, 1216880),
                frac!(3276, 23575),
                frac!(-1142053, 22015140),
                frac!(0, 1),
            ],
        ];

        const AV: &'static [&'static [Ratio]] = &[
            &[],
            &[frac!(8, 39)],
            &[frac!(1, 13), frac!(3, 13)],
            &[frac!(7385, 6912), frac!(-9425, 2304), frac!(13325, 3456)],
            &[
                frac!(223324757, 91364240),
                frac!(-174255393, 18272848),
                frac!(382840094, 46824173),
                frac!(-39627252, 234120865),
            ],
            &[
                frac!(108475, 36464),
                frac!(-9633, 848),
                frac!(7624604, 806183),
                frac!(8100, 49979),
                frac!(-4568212, 19446707),
            ],
            &[
                frac!(4817, 51600),
                frac!(0, 1),
                frac!(1685099, 3650640),
                frac!(19656, 23575),
                frac!(-53676491, 88060560),
                frac!(53, 240),
            ],
        ];

        const BP: &'static [Ratio] = &[
            frac!(4817, 51600),
            frac!(0, 1),
            frac!(388869, 1216880),
            frac!(3276, 23575),
            frac!(-1142053, 22015140),
            frac!(0, 1),
            frac!(0, 1),
        ];

        const BV: &'static [Ratio] = &[
            frac!(4817, 51600),
            frac!(0, 1),
            frac!(1685099, 3650640),
            frac!(19656, 23575),
            frac!(-53676491, 88060560),
            frac!(53, 240),
            frac!(0, 1),
        ];

        const C: &'static [Ratio] = &[
            frac!(0, 1),
            frac!(8, 39),
            frac!(4, 13),
            frac!(5, 6),
            frac!(43, 47),
            frac!(1, 1),
            frac!(1, 1),
        ];
    }
    impl EERKNGCoefficients for Fine45 {
        const ORDER_EMBEDDED: u16 = 5;

        const EP: &'static [Ratio] = &[
            frac!(8151, 2633750),
            frac!(0, 1),
            frac!(-1377519, 186334750),
            frac!(586872, 28879375),
            frac!(-36011118, 2247378875),
            frac!(0, 1),
            frac!(0, 1),
        ];

        const EV: &'static [Ratio] = &[
            frac!(8151, 2633750),
            frac!(0, 1),
            frac!(-5969249, 559004250),
            frac!(3521232, 28879375),
            frac!(-846261273, 4494757750),
            frac!(4187, 36750),
            frac!(-1, 25),
        ];
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct BlanesMoan6B;
    impl<V, P: Problem<State = SecondOrderState<V>>> RKCoefficients<P> for BlanesMoan6B {
        type Instance = SRKN<Self, V>;
    }
    impl SRKNCoefficients for BlanesMoan6B {
        const FSAL: bool = true;

        const A: &[Ratio] = &[
            frac_f64!(0.245298957184271),
            frac_f64!(0.60487266571108),
            frac_f64!(-0.350171622895351),
            frac_f64!(-0.350171622895351),
            frac_f64!(0.60487266571108),
            frac_f64!(0.245298957184271),
            frac_f64!(0.0),
        ];

        const B: &[Ratio] = &[
            frac_f64!(0.0829844064174052),
            frac_f64!(0.396309801498368),
            frac_f64!(-0.0390563049223486),
            frac_f64!(0.1195241940131508),
            frac_f64!(-0.0390563049223486),
            frac_f64!(0.396309801498368),
            frac_f64!(0.0829844064174052),
        ];
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct BlanesMoan11B;
    impl<V, P: Problem<State = SecondOrderState<V>>> RKCoefficients<P> for BlanesMoan11B {
        type Instance = SRKN<Self, V>;
    }
    impl SRKNCoefficients for BlanesMoan11B {
        const FSAL: bool = true;

        const A: &[Ratio] = &[
            frac_f64!(0.123229775946271),
            frac_f64!(0.290553797799558),
            frac_f64!(-0.127049212625417),
            frac_f64!(-0.246331761062075),
            frac_f64!(0.357208872795928),
            frac_f64!(0.20477705429147),
            frac_f64!(0.357208872795928),
            frac_f64!(-0.246331761062075),
            frac_f64!(-0.127049212625417),
            frac_f64!(0.290553797799558),
            frac_f64!(0.123229775946271),
            frac_f64!(0.0),
        ];

        const B: &[Ratio] = &[
            frac_f64!(0.0414649985182624),
            frac_f64!(0.198128671918067),
            frac_f64!(-0.0400061921041533),
            frac_f64!(0.0752539843015807),
            frac_f64!(-0.0115113874206879),
            frac_f64!(0.2366699247869311),
            frac_f64!(0.2366699247869311),
            frac_f64!(-0.0115113874206879),
            frac_f64!(0.0752539843015807),
            frac_f64!(-0.0400061921041533),
            frac_f64!(0.198128671918067),
            frac_f64!(0.0414649985182624),
        ];
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct BlanesMoan14A;
    impl<V, P: Problem<State = SecondOrderState<V>>> RKCoefficients<P> for BlanesMoan14A {
        type Instance = SRKN<Self, V>;
    }
    impl SRKNCoefficients for BlanesMoan14A {
        const FSAL: bool = true;

        const A: &[Ratio] = &[
            frac_f64!(0.0378593198406116),
            frac_f64!(0.102635633102435),
            frac_f64!(-0.0258678882665587),
            frac_f64!(0.314241403071447),
            frac_f64!(-0.130144459517415),
            frac_f64!(0.106417700369543),
            frac_f64!(-0.00879424312851058),
            frac_f64!(0.2073050690568954),
            frac_f64!(-0.00879424312851058),
            frac_f64!(0.106417700369543),
            frac_f64!(-0.130144459517415),
            frac_f64!(0.314241403071447),
            frac_f64!(-0.0258678882665587),
            frac_f64!(0.102635633102435),
            frac_f64!(0.0378593198406116),
        ];

        const B: &[Ratio] = &[
            frac_f64!(0.0),
            frac_f64!(0.09171915262446165),
            frac_f64!(0.183983170005006),
            frac_f64!(-0.05653436583288827),
            frac_f64!(0.004914688774712854),
            frac_f64!(0.143761127168358),
            frac_f64!(0.328567693746804),
            frac_f64!(-0.19641146648645422),
            frac_f64!(-0.19641146648645422),
            frac_f64!(0.328567693746804),
            frac_f64!(0.143761127168358),
            frac_f64!(0.004914688774712854),
            frac_f64!(-0.05653436583288827),
            frac_f64!(0.183983170005006),
            frac_f64!(0.09171915262446165),
        ];
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct ForestRuth;
    impl<V, P: Problem<State = SecondOrderState<V>>> RKCoefficients<P> for ForestRuth {
        type Instance = SRKN<Self, V>;
    }
    impl SRKNCoefficients for ForestRuth {
        const FSAL: bool = true;

        const A: &[Ratio] = &[
            frac_f64!(0.6756035959798288),
            frac_f64!(-0.17560359597982883),
            frac_f64!(-0.17560359597982883),
            frac_f64!(0.6756035959798288),
        ];

        const B: &[Ratio] = &[
            frac_f64!(0.0),
            frac_f64!(1.3512071919596575),
            frac_f64!(-1.7024143839193153),
            frac_f64!(1.3512071919596575),
        ];
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct McLachlanO4;
    impl<V, P: Problem<State = SecondOrderState<V>>> RKCoefficients<P> for McLachlanO4 {
        type Instance = SRKN<Self, V>;
    }
    impl SRKNCoefficients for McLachlanO4 {
        const FSAL: bool = false;

        const A: &[Ratio] = &[
            frac_f64!(0.5153528374311229),
            frac_f64!(-0.08578201941297364),
            frac_f64!(0.4415830236164665),
            frac_f64!(0.12884615836538418),
        ];

        const B: &[Ratio] = &[
            frac_f64!(0.1344961992774311),
            frac_f64!(-0.22481980307942082),
            frac_f64!(0.7563200005156683),
            frac_f64!(0.33400360328632145),
        ];
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct McLachlanSS17;
    impl<V, P: Problem<State = SecondOrderState<V>>> RKCoefficients<P> for McLachlanSS17 {
        type Instance = SRKN<Self, V>;
    }
    impl SRKNCoefficients for McLachlanSS17 {
        const FSAL: bool = true;

        const A: &[Ratio] = &[
            frac_f64!(0.06443298969072164),
            frac_f64!(0.35519003324334714),
            frac_f64!(0.08566935781770041),
            frac_f64!(-0.1125142178766312),
            frac_f64!(-0.11220270385213184),
            frac_f64!(-0.13257320117041968),
            frac_f64!(0.21137072073684585),
            frac_f64!(0.2966460921549873),
            frac_f64!(-0.15601907074441956),
            frac_f64!(-0.15601907074441956),
            frac_f64!(0.2966460921549873),
            frac_f64!(0.21137072073684585),
            frac_f64!(-0.13257320117041968),
            frac_f64!(-0.11220270385213184),
            frac_f64!(-0.1125142178766312),
            frac_f64!(0.08566935781770041),
            frac_f64!(0.35519003324334714),
            frac_f64!(0.06443298969072164),
        ];

        const B: &[Ratio] = &[
            frac_f64!(0.0),
            frac_f64!(0.12886597938144329),
            frac_f64!(0.581514087105251),
            frac_f64!(-0.4101753714698501),
            frac_f64!(0.18514693571658775),
            frac_f64!(-0.4095523434208514),
            frac_f64!(0.14440594108001203),
            frac_f64!(0.27833550039367966),
            frac_f64!(0.31495668391629483),
            frac_f64!(-0.626994825405134),
            frac_f64!(0.31495668391629483),
            frac_f64!(0.27833550039367966),
            frac_f64!(0.14440594108001203),
            frac_f64!(-0.4095523434208514),
            frac_f64!(0.18514693571658775),
            frac_f64!(-0.4101753714698501),
            frac_f64!(0.581514087105251),
            frac_f64!(0.12886597938144329),
        ];
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct Pefrl;
    impl<V, P: Problem<State = SecondOrderState<V>>> RKCoefficients<P> for Pefrl {
        type Instance = SRKN<Self, V>;
    }
    impl SRKNCoefficients for Pefrl {
        const FSAL: bool = true;

        const A: &[Ratio] = {
            const XI: f64 = 0.1786178958448091;
            const CHI: f64 = -0.0662645826698185;

            &[
                frac_f64!(XI),
                frac_f64!(CHI),
                frac_f64!(1.0 - 2.0 * (CHI + XI)),
                frac_f64!(CHI),
                frac_f64!(XI),
            ]
        };

        const B: &[Ratio] = {
            const LAMBDA: f64 = -0.2123418310626054;

            &[
                frac_f64!(0.0),
                frac_f64!(0.5 - LAMBDA),
                frac_f64!(LAMBDA),
                frac_f64!(LAMBDA),
                frac_f64!(0.5 - LAMBDA),
            ]
        };
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct Ruth;
    impl<V, P: Problem<State = SecondOrderState<V>>> RKCoefficients<P> for Ruth {
        type Instance = SRKN<Self, V>;
    }
    impl SRKNCoefficients for Ruth {
        const FSAL: bool = false;

        const A: &[Ratio] = &[frac!(2, 3), frac!(-2, 3), frac!(1, 1)];

        const B: &[Ratio] = &[frac!(7, 24), frac!(3, 4), frac!(-1, 24)];
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct AdamsBashforth2;
    impl<P: Problem> LMCoefficients<P> for AdamsBashforth2 {
        type Instance = ELM1<Self, 2, P::State>;
    }
    impl ELM1Coefficients for AdamsBashforth2 {
        const ALPHA: &'static [i128] = &[1, -1, 0];

        const BETA_N: &'static [i128] = &[0, 3, -1];

        const BETA_D: i128 = 2;
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct AdamsBashforth3;
    impl<P: Problem> LMCoefficients<P> for AdamsBashforth3 {
        type Instance = ELM1<Self, 3, P::State>;
    }
    impl ELM1Coefficients for AdamsBashforth3 {
        const ALPHA: &'static [i128] = &[1, -1, 0, 0];

        const BETA_N: &'static [i128] = &[0, 23, -16, 5];

        const BETA_D: i128 = 12;
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct AdamsBashforth4;
    impl<P: Problem> LMCoefficients<P> for AdamsBashforth4 {
        type Instance = ELM1<Self, 4, P::State>;
    }
    impl ELM1Coefficients for AdamsBashforth4 {
        const ALPHA: &'static [i128] = &[1, -1, 0, 0, 0];

        const BETA_N: &'static [i128] = &[0, 55, -59, 37, -9];

        const BETA_D: i128 = 24;
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct AdamsBashforth5;
    impl<P: Problem> LMCoefficients<P> for AdamsBashforth5 {
        type Instance = ELM1<Self, 5, P::State>;
    }
    impl ELM1Coefficients for AdamsBashforth5 {
        const ALPHA: &'static [i128] = &[1, -1, 0, 0, 0, 0];

        const BETA_N: &'static [i128] = &[0, 1901, -2774, 2616, -1274, 251];

        const BETA_D: i128 = 720;
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct AdamsBashforth6;
    impl<P: Problem> LMCoefficients<P> for AdamsBashforth6 {
        type Instance = ELM1<Self, 6, P::State>;
    }
    impl ELM1Coefficients for AdamsBashforth6 {
        const ALPHA: &'static [i128] = &[1, -1, 0, 0, 0, 0, 0];

        const BETA_N: &'static [i128] = &[0, 4277, -7923, 9982, -7298, 2877, -475];

        const BETA_D: i128 = 1440;
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct QuinlanTremaine12;
    impl<V, P: Problem<State = SecondOrderState<V>>> LMCoefficients<P> for QuinlanTremaine12 {
        type Instance = ELM2<Self, 12, V>;
    }
    impl ELM2Coefficients for QuinlanTremaine12 {
        const ALPHA: &'static [i128] = &[1, -2, 2, -1, 0, 0, 0, 0, 0, -1, 2, -2, 1];

        const BETA_N: &'static [i128] = &[
            0,
            90987349,
            -229596838,
            812627169,
            -1628539944,
            2714971338,
            -3041896548,
            2714971338,
            -1628539944,
            812627169,
            -229596838,
            90987349,
            0,
        ];

        const BETA_D: i128 = 53222400;
    }

    #[doc(hidden)]
    #[derive(Clone, Copy, Debug)]
    pub struct Stormer13;
    impl<V, P: Problem<State = SecondOrderState<V>>> LMCoefficients<P> for Stormer13 {
        type Instance = ELM2<Self, 13, V>;
    }
    impl ELM2Coefficients for Stormer13 {
        const ALPHA: &'static [i128] = &[1, -2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        const BETA_N: &'static [i128] = &[
            0,
            4_6211_5547_1343,
            -13_2328_4191_4856,
            47_0137_4372_6958,
            -114_3217_0067_2600,
            202_2719_6761_1865,
            -266_6095_4958_4656,
            264_4290_2189_5332,
            -197_1068_0827_6656,
            108_9829_3333_3425,
            -43_4275_9282_8040,
            11_8071_4397_8638,
            -1_9627_7757_4776,
            1506_5357_0023,
        ];

        const BETA_D: i128 = 2_6153_4873_6000;
    }
}
