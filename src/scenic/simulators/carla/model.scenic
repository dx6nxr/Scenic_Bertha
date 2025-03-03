"""Scenic world model for traffic scenarios in CARLA.

The model currently supports vehicles, pedestrians, and props. It implements the
basic `Car` and `Pedestrian` classes from the :obj:`scenic.domains.driving` domain,
while also providing convenience classes for specific types of objects like bicycles,
traffic cones, etc. Vehicles and pedestrians support the basic actions and behaviors
from the driving domain; several more are automatically imported from
:obj:`scenic.simulators.carla.actions` and :obj:`scenic.simulators.carla.behaviors`.

The model defines several global parameters, whose default values can be overridden
in scenarios using the ``param`` statement or on the command line using the
:option:`--param` option:

Global Parameters:
    carla_map (str): Name of the CARLA map to use, e.g. 'Town01'. Can also be set
        to ``None``, in which case CARLA will attempt to create a world from the
        **map** file used in the scenario (which must be an ``.xodr`` file).
    timestep (float): Timestep to use for simulations (i.e., how frequently Scenic
        interrupts CARLA to run behaviors, check requirements, etc.), in seconds. Default
        is 0.1 seconds.
    snapToGroundDefault (bool): Default value for :prop:`snapToGround` on `CarlaActor` objects.
        Default is True if :ref:`2D compatibility mode` is enabled and False otherwise. 

    weather (str or dict): Weather to use for the simulation. Can be either a
        string identifying one of the CARLA weather presets (e.g. 'ClearSunset') or a
        dictionary specifying all the weather parameters (see `carla.WeatherParameters`_).
        Default is a uniform distribution over all the weather presets.

    address (str): IP address at which to connect to CARLA. Default is localhost
        (127.0.0.1).
    port (int): Port on which to connect to CARLA. Default is 2000.
    timeout (float): Maximum time to wait when attempting to connect to CARLA, in
        seconds. Default is 10.

    render (int): Whether or not to have CARLA create a window showing the
        simulations from the point of view of the ego object: 1 for yes, 0
        for no. Default 1.
    record (str): If nonempty, folder in which to save CARLA record files for
        replaying the simulations.

.. _carla.WeatherParameters: https://carla.readthedocs.io/en/latest/python_api/#carlaweatherparameters

"""
import pathlib
from scenic.domains.driving.model import *
from carla import Location, Vector3D
from carla import Color as carlaColor

import scenic.simulators.carla.blueprints as blueprints
from scenic.simulators.carla.behaviors import *
from scenic.simulators.utils.colors import Color
from shapely.geometry import Polygon
from carla.libcarla import CityObjectLabel
import math

try:
    from scenic.simulators.carla.simulator import CarlaSimulator    # for use in scenarios
    from scenic.simulators.carla.actions import *
    from scenic.simulators.carla.actions import _CarlaVehicle, _CarlaPedestrian
    import scenic.simulators.carla.utils.utils as _utils
except ModuleNotFoundError:
    # for convenience when testing without the carla package
    from scenic.core.simulators import SimulatorInterfaceWarning
    import warnings
    warnings.warn('the "carla" package is not installed; '
                  'will not be able to run dynamic simulations',
                  SimulatorInterfaceWarning)

    def CarlaSimulator(*args, **kwargs):
        """Dummy simulator to allow compilation without the 'carla' package.

        :meta private:
        """
        raise RuntimeError('the "carla" package is required to run simulations '
                           'from this scenario')

    class _CarlaVehicle: pass
    class _CarlaPedestrian: pass

map_town = pathlib.Path(globalParameters.map).stem
param carla_map = map_town
param address = '127.0.0.1'
param port = 2000
param timeout = 10
param render = 1
if globalParameters.render not in [0, 1]:
    raise ValueError('render param must be either 0 or 1')
param record = ''
param timestep = 0.1
param weather = Uniform(
    'ClearNoon',
    'CloudyNoon',
    'WetNoon',
    'WetCloudyNoon',
    'SoftRainNoon',
    'MidRainyNoon',
    'HardRainNoon',
    'ClearSunset',
    'CloudySunset',
    'WetSunset',
    'WetCloudySunset',
    'SoftRainSunset',
    'MidRainSunset',
    'HardRainSunset'
)
param snapToGroundDefault = is2DMode()

simulator CarlaSimulator(
    carla_map=globalParameters.carla_map,
    map_path=globalParameters.map,
    address=globalParameters.address,
    port=int(globalParameters.port),
    timeout=int(globalParameters.timeout),
    render=bool(globalParameters.render),
    record=globalParameters.record,
    timestep=float(globalParameters.timestep)
)

class CarlaActor(DrivingObject):
    """Abstract class for CARLA objects.

    Properties:
        carlaActor (dynamic): Set during simulations to the ``carla.Actor`` representing this
            object.
        blueprint (str): Identifier of the CARLA blueprint specifying the type of object.
        rolename (str): Can be used to differentiate specific actors during runtime. Default
            value ``None``.
        physics (bool): Whether physics is enabled for this object in CARLA. Default true.
        snapToGround (bool): Whether or not to snap this object to the ground when placed in CARLA.
            The default is set by the ``snapToGroundDefault`` global parameter above.
    """
    carlaActor: None
    blueprint: None
    rolename: None
    color: None
    physics: True
    snapToGround: globalParameters.snapToGroundDefault

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._control = None    # used internally to accumulate control updates

    @property
    def control(self):
        if self._control is None:
            self._control = self.carlaActor.get_control()
        return self._control

    def setPosition(self, pos, elevation):
        self.carlaActor.set_location(_utils.scenicToCarlaLocation(pos, elevation))

    def setVelocity(self, vel):
        cvel = _utils.scenicToCarlaVector3D(*vel)
        if hasattr(self.carlaActor, 'set_target_velocity'):
            self.carlaActor.set_target_velocity(cvel)
        else:
            self.carlaActor.set_velocity(cvel)

class Vehicle(Vehicle, CarlaActor, Steers, _CarlaVehicle):
    """Abstract class for steerable vehicles."""

    def setThrottle(self, throttle):
        self.control.throttle = throttle

    def setSteering(self, steering):
        self.control.steer = steering

    def setBraking(self, braking):
        self.control.brake = braking

    def setHandbrake(self, handbrake):
        self.control.hand_brake = handbrake

    def setReverse(self, reverse):
        self.control.reverse = reverse

    def _getClosestTrafficLight(self, distance=100):
        return _getClosestTrafficLight(self, distance)

class Car(Vehicle):
    """A car.

    The default ``blueprint`` (see `CarlaActor`) is a uniform distribution over the
    blueprints listed in :obj:`scenic.simulators.carla.blueprints.carModels`.
    """
    blueprint: Uniform(*blueprints.carModels)

    @property
    def isCar(self):
        return True

class NPCCar(Car):  # no distinction between these in CARLA
    pass

class Bicycle(Vehicle):
    width: 1
    length: 2
    blueprint: Uniform(*blueprints.bicycleModels)


class Motorcycle(Vehicle):
    width: 1
    length:2
    blueprint: Uniform(*blueprints.motorcycleModels)


class Truck(Vehicle):
    width: 3
    length: 7
    blueprint: Uniform(*blueprints.truckModels)


class Pedestrian(Pedestrian, CarlaActor, Walks, _CarlaPedestrian):
    """A pedestrian.

    The default ``blueprint`` (see `CarlaActor`) is a uniform distribution over the
    blueprints listed in :obj:`scenic.simulators.carla.blueprints.walkerModels`.
    """
    width: 0.5
    length: 0.5
    blueprint: Uniform(*blueprints.walkerModels)
    carlaController: None

    def setWalkingDirection(self, heading):
        direction = Vector(0, 1, 0).rotatedBy(heading)
        self.control.direction = _utils.scenicToCarlaVector3D(*direction)

    def setWalkingSpeed(self, speed):
        self.control.speed = speed


class Prop(CarlaActor):
    """Abstract class for props, i.e. non-moving objects.

    Properties:
        parentOrientation (Orientation): Default value overridden to have uniformly random yaw.
        physics (bool): Default value overridden to be false.
    """
    regionContainedIn: road
    position: new Point on road
    parentOrientation: Range(0, 360) deg
    width: 0.5
    length: 0.5
    physics: False

class Trash(Prop):
    blueprint: Uniform(*blueprints.trashModels)


class Cone(Prop):
    blueprint: Uniform(*blueprints.coneModels)


class Debris(Prop):
    blueprint: Uniform(*blueprints.debrisModels)


class VendingMachine(Prop):
    blueprint: Uniform(*blueprints.vendingMachineModels)


class Chair(Prop):
    blueprint: Uniform(*blueprints.chairModels)


class BusStop(Prop):
    blueprint: Uniform(*blueprints.busStopModels)


class Advertisement(Prop):
    blueprint: Uniform(*blueprints.advertisementModels)


class Garbage(Prop):
    blueprint: Uniform(*blueprints.garbageModels)


class Container(Prop):
    blueprint: Uniform(*blueprints.containerModels)


class Table(Prop):
    blueprint: Uniform(*blueprints.tableModels)


class Barrier(Prop):
    blueprint: Uniform(*blueprints.barrierModels)


class PlantPot(Prop):
    blueprint: Uniform(*blueprints.plantpotModels)


class Mailbox(Prop):
    blueprint: Uniform(*blueprints.mailboxModels)


class Gnome(Prop):
    blueprint: Uniform(*blueprints.gnomeModels)


class CreasedBox(Prop):
    blueprint: Uniform(*blueprints.creasedboxModels)


class Case(Prop):
    blueprint: Uniform(*blueprints.caseModels)


class Box(Prop):
    blueprint: Uniform(*blueprints.boxModels)


class Bench(Prop):
    blueprint: Uniform(*blueprints.benchModels)


class Barrel(Prop):
    blueprint: Uniform(*blueprints.barrelModels)


class ATM(Prop):
    blueprint: Uniform(*blueprints.atmModels)


class Kiosk(Prop):
    blueprint: Uniform(*blueprints.kioskModels)


class IronPlate(Prop):
    blueprint: Uniform(*blueprints.ironplateModels)


class TrafficWarning(Prop):
    blueprint: Uniform(*blueprints.trafficwarningModels)


## Utility functions

def freezeTrafficLights():
    """Freezes all traffic lights in the scene.

    Frozen traffic lights can be modified by the user
    but the time will not update them until unfrozen.
    """
    simulation().world.freeze_all_traffic_lights(True)

def unfreezeTrafficLights():
    """Unfreezes all traffic lights in the scene."""
    simulation().world.freeze_all_traffic_lights(False)

def setAllIntersectionTrafficLightsStatus(intersection, color):
    for signal in intersection.signals:
        if signal.isTrafficLight:
            setTrafficLightStatus(signal, color)

def setTrafficLightStatus(signal, color):
    if not signal.isTrafficLight:
        raise RuntimeError('The provided signal is not a traffic light')
    color = utils.scenicToCarlaTrafficLightStatus(color)
    if color is None:
        raise RuntimeError('Color must be red/yellow/green/off/unknown.')
    landmarks = simulation().map.get_all_landmarks_from_id(signal.openDriveID)
    if landmarks:
        traffic_light = simulation().world.get_traffic_light(landmarks[0])
        if traffic_light is not None:
            traffic_light.set_state(color)

def getTrafficLightStatus(signal):
    if not signal.isTrafficLight:
        raise RuntimeError('The provided signal is not a traffic light')
    landmarks = simulation().map.get_all_landmarks_from_id(signal.openDriveID)
    if landmarks:
        traffic_light = simulation().world.get_traffic_light(landmarks[0])
        if traffic_light is not None:
            return utils.carlaToScenicTrafficLightStatus(traffic_light.state)
    return "None"

def _getClosestLandmark(vehicle, type, distance=100):
    if vehicle._intersection is not None:
        return None

    waypoint = simulation().map.get_waypoint(vehicle.carlaActor.get_transform().location)
    landmarks = waypoint.get_landmarks_of_type(distance, type)

    if landmarks:
        return min(landmarks, key=lambda l: l.distance)
    return None

def _getClosestTrafficLight(vehicle, distance=100):
    """Returns the closest traffic light affecting 'vehicle', up to a maximum of 'distance'"""
    landmark = _getClosestLandmark(vehicle, type="1000001", distance=distance)
    if landmark is not None:
        return simulation().world.get_traffic_light(landmark)
    return None

def withinDistanceToRedYellowTrafficLight(vehicle, thresholdDistance):
    traffic_light = _getClosestTrafficLight(vehicle, distance=thresholdDistance)
    if traffic_light is not None and str(traffic_light.state) in ("Red", "Yellow"):
        return True
    return False

def withinDistanceToTrafficLight(vehicle, thresholdDistance):
    traffic_light = _getClosestTrafficLight(vehicle, distance=thresholdDistance)
    if traffic_light is not None:
        return True
    return False

def getClosestTrafficLightStatus(vehicle, distance=100):
    traffic_light = _getClosestTrafficLight(vehicle, distance)
    if traffic_light is not None:
        return _utils.carlaToScenicTrafficLightStatus(traffic_light.state)
    return "None"

def setClosestTrafficLightStatus(vehicle, color, distance=100):
    color = _utils.scenicToCarlaTrafficLightStatus(color)
    if color is None:
        raise RuntimeError('Color must be red/yellow/green/off/unknown.')
    
    traffic_light = _getClosestTrafficLight(vehicle, distance)
    if traffic_light is not None:
        traffic_light.set_state(color)

def logNearestTrafficLight(vehicle, logger, state, timestep, distance=100):
    traffic_light = _getClosestTrafficLight(vehicle, distance)
    if traffic_light is not None:
        logger.logTrafficLightData(traffic_light.get_location(), timestep, state)

def logIntersectionTrafficLights(intersection, logger, state, timestep):
    for signal in intersection.signals:
        if signal.isTrafficLight:
            landmarks = simulation().map.get_all_landmarks_from_id(signal.openDriveID)
            traffic_light = simulation().world.get_traffic_light(landmarks[0])
            logger.logTrafficLightData(traffic_light.get_location(), timestep, state)

def logTrafficLightCondition(actor, logger, timestep, distance=100):
    traffic_light = _getClosestTrafficLight(actor, distance)
    if traffic_light is not None:
        logger.logTrafficLightCondition(traffic_light.get_location(), timestep, actor.rolename)


def pedestrianVisibilityPercentageFromDriver(vehicle, pedestrian, max_distance=100, fov_left_angle=55, fov_right_angle=75):
    """
    Calculate the visibility percentage of a pedestrian from the driver's perspective.

    Args:
        vehicle: The scenic vehicle actor
        pedestrian: The scenic pedestrian actor
        max_distance: Maximum distance to consider
        fov_left_angle: Left angle of the field of view in degrees
        fov_right_angle: Right angle of the field of view in degrees

    Returns:
        (float): Visibility percentage (0-1) corresponding to 0% to 100% visibility
    """
    # Get vehicle and pedestrian locations
    ego_transform = vehicle.carlaActor.get_transform()
    ego_location = ego_transform.location
    ped_location = pedestrian.carlaActor.get_location()
    
    # Quick distance check - return 0 if beyond max_distance
    if ego_location.distance(ped_location) > max_distance:
        return 0.0
    
    # Set up driver's eye position
    ego_bb = vehicle.carlaActor.bounding_box
    ego_bb_extent = ego_bb.extent
    ego_location.z += ego_bb_extent.z*1.5
    
    # Move the location to the driver seat
    ego_yaw_rad = math.radians(ego_transform.rotation.yaw)
    ego_location.x += ego_bb_extent.x * 0.1 * math.cos(ego_yaw_rad) - ego_bb_extent.y * -0.4 * math.sin(ego_yaw_rad)
    ego_location.y += ego_bb_extent.x * 0.1 * math.sin(ego_yaw_rad) + ego_bb_extent.y * -0.4 * math.cos(ego_yaw_rad)

    # Calculate FOV directions
    fov_data = calculateFOV(ego_location, ego_transform, fov_left_angle, fov_right_angle, max_distance)
    
    # Cast rays and calculate visibility percentage
    visibility_percentage = calculateVisibility(
        ego_location, 
        vehicle.carlaActor, 
        ego_transform,
        pedestrian.carlaActor,
        fov_data,
        max_distance
    )
    
    return visibility_percentage


def calculateFOV(origin_location, origin_transform, left_angle, right_angle, max_length):
    """Calculates and visualizes field of view based on given parameters.
    
    Args:
        origin_location: Location from which the FOV originates 
        origin_transform: Transform of the origin object
        left_angle: Maximum angle to the left in degrees
        right_angle: Maximum angle to the right in degrees
        max_length: Maximum length of the FOV
        
    Returns:
        Dictionary containing FOV data:
        - forward_vector: Normalized forward vector
        - left_direction: Direction vector for left boundary
        - right_direction: Direction vector for right boundary
        - left_angle: Left FOV angle in degrees
        - right_angle: Right FOV angle in degrees
    """
    # Calculate FOV directions
    forward_vector = origin_transform.get_forward_vector()
    forward_vector_normalized = Vector3D(forward_vector.x, forward_vector.y, 0).make_unit_vector()
    
    # Convert angles to radians
    left_rad = math.radians(-left_angle)
    right_rad = math.radians(right_angle)
    
    # Left FOV boundary
    sin_left = math.sin(left_rad)
    cos_left = math.cos(left_rad)
    left_direction = Vector3D(
        forward_vector.x * cos_left - forward_vector.y * sin_left,
        forward_vector.x * sin_left + forward_vector.y * cos_left,
        forward_vector.z
    )

    # Right FOV boundary
    sin_right = math.sin(right_rad)
    cos_right = math.cos(right_rad)
    right_direction = Vector3D(
        forward_vector.x * cos_right - forward_vector.y * sin_right,
        forward_vector.x * sin_right + forward_vector.y * cos_right,
        forward_vector.z
    )

    # Draw FOV visualization
    simulation().world.debug.draw_arrow(
        origin_location, 
        origin_location + Location(left_direction.x * max_length, left_direction.y * max_length, left_direction.z * max_length), 
        thickness=0.05, 
        arrow_size=0.1, 
        color=carlaColor(0, 0, 255), 
        life_time=0.5
    )
    
    simulation().world.debug.draw_arrow(
        origin_location, 
        origin_location + Location(right_direction.x * max_length, right_direction.y * max_length, right_direction.z * max_length), 
        thickness=0.05, 
        arrow_size=0.1, 
        color=carlaColor(0, 0, 255), 
        life_time=0.5
    )
    
    # Return FOV data
    return {
        "forward_vector": forward_vector_normalized,
        "left_direction": left_direction,
        "right_direction": right_direction,
        "left_angle": left_angle,
        "right_angle": right_angle
    }


def calculateVisibility(ray_origin, ego_actor, ego_transform, target_actor, fov_data, max_distance, num_rays=250):
    """Calculates visibility percentage based on ray casting within the specified FOV.
    
    Args:
        ray_origin: Location from which rays originate
        ego_actor: The actor from which we're casting rays
        ego_transform: Transform of the ego actor
        target_actor: The actor we're testing visibility of
        fov_data: Dictionary with FOV parameters from calculateFOV
        max_distance: Maximum distance to cast rays
        num_rays: Number of rays to cast
        
    Returns:
        Visibility percentage (0-1) corresponding to 0% to 100% visibility
    """
    GRID_SIZE = int(math.sqrt(num_rays))
    
    # Extract FOV data
    forward_vector = fov_data["forward_vector"]
    left_angle = fov_data["left_angle"]
    right_angle = fov_data["right_angle"]
    
    # Prepare pedestrian bounding box
    target_bb = target_actor.bounding_box
    target_bb.extent += Vector3D(0.15, 0.15, 0.15)  # Extend for hands
    target_transform = target_actor.get_transform()
    
    # Get corners for calculating dimensions
    corners = target_bb.get_world_vertices(target_transform)
    
    # Calculate bounding box dimensions
    min_x = min(corner.x for corner in corners)
    max_x = max(corner.x for corner in corners)
    min_y = min(corner.y for corner in corners)
    max_y = max(corner.y for corner in corners)
    min_z = min(corner.z for corner in corners)
    max_z = max(corner.z for corner in corners)
    
    width = max_x - min_x
    height = max_z - min_z
    avg_y = (max_y + min_y) / 2
    
    # Cache vehicle bounding box for optimization
    ego_bb = ego_actor.bounding_box
    
    # Counters
    num_hits = 0
    num_occluded = 0
    num_outside_fov = 0
    
    # Ray casting loop with grid-based sampling
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            # Sample point on pedestrian
            x = min_x + i * width / (GRID_SIZE - 1)
            z = min_z + j * height / (GRID_SIZE - 1)
            ray_end = Location(x, avg_y, z)
            
            # Calculate ray direction vector
            ray_direction = Vector3D(ray_end.x - ray_origin.x, ray_end.y - ray_origin.y, 0)
            ray_length = math.sqrt(ray_direction.x**2 + ray_direction.y**2)
            
            # Skip if beyond max distance
            if ray_length > max_distance:
                num_outside_fov += 1
                continue
                
            ray_direction = ray_direction.make_unit_vector()
            
            # Calculate angle with forward vector
            dot_product = ray_direction.x * forward_vector.x + ray_direction.y * forward_vector.y
            angle = math.degrees(math.acos(max(-1.0, min(1.0, dot_product))))
            
            # Check if ray is left/right of forward vector using cross product
            cross_product_z = forward_vector.x * ray_direction.y - forward_vector.y * ray_direction.x
            is_left = cross_product_z > 0
            
            # Check if ray is within FOV
            is_in_fov = (is_left and angle <= left_angle) or (not is_left and angle <= right_angle)
            
            if not is_in_fov:
                num_outside_fov += 1
                continue
            
            # Cast ray
            hit_list = simulation().world.cast_ray(ray_origin, ray_end)
            hit_list = [hit for hit in hit_list if hit.label != CityObjectLabel.NONE and not ego_bb.contains(hit.location, ego_transform)]
            
            if hit_list:
                first_hit = hit_list[0]
                
                # If first hit is on the pedestrian, count as direct hit
                if first_hit.label == CityObjectLabel.Pedestrians and target_bb.contains(first_hit.location, target_transform):
                    num_hits += 1
                else:
                    # Check if any subsequent hit is on the pedestrian (occluded)
                    for hit in hit_list:
                        if hit.label == CityObjectLabel.Pedestrians and target_bb.contains(hit.location, target_transform):
                            num_occluded += 1
                            break
    
    # Calculate visibility percentage
    total_rays_in_fov = num_rays - num_outside_fov
    
    if total_rays_in_fov > 0 and (num_hits + num_occluded) > 0:
        visibility_percentage = (num_hits / (num_hits + num_occluded))
    else:
        visibility_percentage = 0.0
        
    return visibility_percentage