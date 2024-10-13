from skyfield.api import Loader
from skyfield.iokit import _NAIF_KERNELS, _NAIF, _JPL
from skyfield.planetarylib import PlanetaryConstants
from skyfield.jpllib import _jpl_code_name_dict, _jpl_name_code_dict

class Constants:
    def __init__(self, id, name, mu):
        self.id = id
        self.name = name
        self.mu = mu

class AlmanacResult:
    def __init__(self, state, constants):
        self.state = state
        self.constants = constants

class Almanac:
    def __init__(self, directory, start_date, end_date=None):
        if end_date is None:
            end_date = start_date

        assert start_date.ts == end_date.ts

        loader = Loader(directory)
        loader.urls = {
            '.tpc': [
                ('*.tpc', _NAIF_KERNELS + 'pck/'),
            ],
            '.bsp': [
                ('de*.bsp', _JPL),
                ('*.bsp', _NAIF),
            ],
        }

        ephs = {
         **{i: 'de440s.bsp' for i in range(1, 4)},
            4: 'mar097.bsp', 
            5: 'jup365.bsp', 
            6: 'sat441.bsp', 
            7: 'ura111.bsp', 
            8: 'nep097.bsp', 
            9: 'plu060.bsp',
        }

        pc = PlanetaryConstants()
        pc.read_text(loader('gm_de440.tpc'))
        pc.read_text(loader('pck00011.tpc'))

        self._start_date = start_date
        self._end_date = end_date
        self._ts = start_date.ts
        self._loader = loader
        
        self.ephs = ephs
        self.pc = pc
    
    def __getitem__(self, item):
        if isinstance(item, str):
            item = _jpl_name_code_dict[item.upper()]

        # We could use jplephem to only get the segments that are relevant to the date range
        eph = self._loader(self.ephs[int(str(item)[:1])])
        # We need this because Skyfield does not correctly support ephemerides with multiple segments 
        # Which is also why we need a start and end date to create the Almanac even though we load the entire ephemeris...
        # (https://github.com/skyfielders/python-skyfield/issues/691)
        eph.segments = [
            segment for segment in eph.segments 
                if segment.time_range(self._ts)[0] < self._start_date
                and segment.time_range(self._ts)[1] > self._end_date
        ]
        if not eph.segments:
            raise ValueError('No segments found for the date range')

        state = eph[item]
        constants = Constants(
            item,
            _jpl_code_name_dict[item].title(),
            self.pc.variables['BODY{0}_GM'.format(item)]
        )

        return AlmanacResult(state, constants)
