from almanac import Almanac
import skyfield
import json
import os
import sys

ts = skyfield.api.load.timescale()
epoch = ts.tai(1950, 1, 1, 0)
bodies = [
    'Sun',
    'Mercury',
    'Venus',
    'Earth',
    'Moon',
    'Mars',
    'Phobos',
    'Deimos',
    'Jupiter',
    'Io',
    'Europa',
    'Ganymede',
    'Callisto',
    'Saturn',
    'Mimas',
    'Enceladus',
    'Tethys',
    'Dione',
    'Rhea',
    'Titan',
    'Iapetus',
    'Uranus',
    'Ariel',
    'Umbriel',
    'Titania',
    'Oberon',
    'Miranda',
    'Neptune',
    'Triton',
    'Pluto',
    'Charon'
]
# bodies = [
#     'Sun',
#     'Mercury',
#     'Venus',
#     'Earth',
#     'Moon',
#     'Mars Barycenter',
#     'Jupiter Barycenter',
#     'Saturn Barycenter',
#     'Uranus Barycenter',
#     'Neptune Barycenter',
# ]

if __name__ == '__main__':
    name = sys.argv.pop()
    file = './output/{0}_{1}.json'.format(name, epoch.tdb)
    os.makedirs(os.path.dirname(file), exist_ok=True)
    
    with open(file, 'w') as f:
        almanac = Almanac('./data', epoch)
        ephs = map(lambda body: almanac[body], bodies)
        data = list(map(lambda r: {
            'id': r.constants.id,
            'name': r.constants.name,
            'mu': r.constants.mu,
            'position': r.state.at(epoch).position.km.tolist(),
            'velocity': r.state.at(epoch).velocity.km_per_s.tolist(),
        }, ephs))

        json.dump({
            'epoch': epoch.tai_strftime(),
            'bodies': data
        }, f, indent=4)