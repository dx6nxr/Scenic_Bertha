param map = localPath('../../assets/maps/CARLA/Town05.xodr')

model scenic.simulators.carla.model

leichterRegen = dict(cloudiness=60.0, precipitation=20, precipitation_deposits=30.0, wind_intensity=10.0, sun_azimuth_angle=90.0,
                     sun_altitude_angle=45.0, fog_density=0.0, fog_distance=50.0, fog_falloff=0.0, wetness=20.0, scattering_intensity=0.0, mie_scattering_scale=0.0, rayleigh_scattering_scale=0.03310000151395798, dust_storm=0.0)
param weather = Uniform(*["ClearSunset", "ClearNoon", leichterRegen])

laneSections = []
for laneSection in network.laneSections:
    if laneSection._laneToLeft != None and laneSection._fasterLane == None:
        laneSections.append(laneSection)

laneSection = Uniform(*laneSections)
parkingPoint = new OrientedPoint on laneSection.laneToRight.centerline

egoLane = laneSection.lane

egoTrajectory = [egoLane.predecessor, egoLane, egoLane.successor]

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=Range(8, 10)) until distance from self to pedestrian < 15
    while pedestrian not in egoLane.group.opposite:
        take SetBrakeAction(1.0)
    do FollowLaneBehavior(target_speed=Range(8, 10))

behavior ParkedCarBehavior():
    take SetHandBrakeAction(True)

ego = new Car on egoLane.predecessor.centerline,
    with behavior EgoBehavior(),
    with rolename 'ego',
    with blueprint "vehicle.tesla.model3"

parkedCar1 = new Car at parkingPoint,
    with behavior ParkedCarBehavior(),
    with rolename 'parkedCar1'

pedestrian = new Pedestrian behind parkedCar1 by 2,
    with behavior CrossingBehavior(ego, min_speed=0.5, threshold=30),
    with regionContainedIn None,
    with rolename 'pedestrian',
    with heading 90 deg relative to parkedCar1.heading

parkedCar2 = new Car following roadDirection from parkedCar1 for -10,
    with behavior ParkedCarBehavior(),
    with rolename 'parkedCar2'

parkedCar3 = new Car following roadDirection from parkedCar2 for -5,
    with behavior ParkedCarBehavior(),
    with rolename 'parkedCar3'

record pedestrianVisibilityPercentageFromDriver(ego, pedestrian) as 'pedestrianVisibilityPercentageFromDriver'
require eventually(pedestrian in egoLane)
require distance from ego to pedestrian > 25
require always distance from ego to pedestrian > 2
require distance from parkedCar1 to intersection > 5
require distance from parkedCar3 to intersection > 5
require eventually distance from ego to parkedCar1 > 30
#require ego.blueprint is not "vehicle.dodge.charger_police" and not "vehicle.nissan.patrol" and not "vehicle.nissan.micra" and not "vehicle.seat.leon" and not "vehicle.audi.a2"
terminate after 15 seconds
require eventually distance from ego to pedestrian < 10
