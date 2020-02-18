import numpy as np
import random
import keyboard
import Box2D
from Box2D.b2 import world, polygonShape, circleShape, edgeShape, staticBody, dynamicBody, kinematicBody, revoluteJoint, wheelJoint, contact
from Box2D import b2Vec2, b2FixtureDef, b2PolygonShape, b2CircleShape, b2Dot, b2EdgeShape, b2Contact, b2ContactFilter, b2Filter, b2ContactListener

import pygame
from pygame import HWSURFACE, DOUBLEBUF, RESIZABLE, VIDEORESIZE
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
pygame.init()


# PyBox2D setup
PPM = 5
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480
POS_X = SCREEN_WIDTH / PPM / 3
POS_Y = SCREEN_HEIGHT / PPM / 3

CAR_CATEGORY = 0x0002
PEDESTRIAN_CATEGORY = 0x0004
BUILDING_CATEGORY = 0x0008

CAR_GROUP = 2
PEDESTRIAN_GROUP = -4
BUILDING_GROUP = -8

contact_flag = False
class myContactListener(b2ContactListener):
    def handle_contact(self, contact, began):
        # A contact happened -- see if a wheel hit a
        # ground area
        fixture_a = contact.fixtureA
        fixture_b = contact.fixtureB

        body_a, body_b = fixture_a.body, fixture_b.body
        ud_a, ud_b = body_a.userData, body_b.userData

        pedestrian_contact = None
        car_contact = None
        building_contact = None
        wheel_contact = None

        for ud in (ud_a, ud_b):
            if ud is not None:
                obj = ud['obj']
                if isinstance(obj, Car):
                    car_contact = obj
                elif isinstance(obj, Pedestrian):
                    pedestrian_contact = obj
                elif isinstance(obj, Building):
                    building_contact = obj
                elif isinstance(obj, Wheel):
                    wheel_contact = obj

        if car_contact is not None and pedestrian_contact is not None:
            print("Shame on you, you killed an innocent pedestrian")
            _destroy()

        if car_contact  is not None and building_contact is not None:
            print("BUMMM")
            _destroy()

    def __init__(self):
        b2ContactListener.__init__(self)

    def BeginContact(self, contact):
        self.handle_contact(contact, True)


    def EndContact(self, contact):
        pass

    def PreSolve(self, contact, oldManifold):
        pass

    def PostSolve(self, contact, impulse):
        pass

def _destroy():
    for building in skyscrapers:
        building.destroy_flag = True


    for pedestrian in johnnie_walkers:
        pedestrian.destroy_flag = True

    for wheel in cars[0].tires:
        wheel.destroy_flag = True

    cars[0].destroy_flag = True


def _reset():
    if len(box2world.bodies) == 0:
        for building in skyscrapers:
            building.destroy_flag = False

        for pedestrian in johnnie_walkers:
            pedestrian.destroy_flag = False

        for wheel in cars[0].tires:
            wheel.destroy_flag = False
    
        cars[0].destroy_flag = False
        
        create_buildings()      
        create_pedestrians()
        create_car()
        cars[0].control()

box2world = world(contactListener=myContactListener(), gravity=(0.0, 0.0), doSleep=True)


class Wheel():
    def __init__(self, box2world, phi, max_lateral_impulse=1, shape=(0.25, 0.75), turn_torque=15000, position=None):

        if position is None:
            position = [POS_X, POS_Y]

        self.destroy_flag = False
        self.position = position
        self.shape = shape
        self.box2world = box2world
        self.phi = phi
        self.max_lateral_impulse = max_lateral_impulse
        self.turn_torque = turn_torque
        self.wheel = self.box2world.CreateDynamicBody(position=(self.position[0], self.position[1]),
                                                      angle=self.phi,
                                                      fixtures=b2FixtureDef(
                                                      shape=b2PolygonShape(box=shape),
                                                      density=650.0,
                                                      friction=10000,
                                                      ))
        self.wheel.userData = {'obj': self}

    # Cancelling lateral velocity

    @property
    def forward_velocity(self):
        current_normal = self.wheel.GetWorldVector((0, 1))
        return current_normal.dot(self.wheel.linearVelocity) * current_normal

    def get_lateral_velocity(self):
        currentRightNormal = self.wheel.GetWorldVector((1, 0))
        return currentRightNormal.dot(self.wheel.linearVelocity) * currentRightNormal

    def apply_impulse(self):
        Impulse = self.wheel.mass * -Wheel.get_lateral_velocity(self)
        self.wheel.ApplyLinearImpulse(Impulse, self.wheel.worldCenter, wake=True)

        if Impulse.length > self.max_lateral_impulse:
            Impulse *= self.max_lateral_impulse / Impulse.length

            self.wheel.ApplyLinearImpulse(Impulse, self.wheel.worldCenter, True)

        # Allowing skidding
        angular_impulse = 0.5 * self.wheel.inertia * -self.wheel.angularVelocity
        self.wheel.ApplyAngularImpulse(angular_impulse, True)

        current_forward_normal = self.forward_velocity
        current_forward_speed = current_forward_normal.Normalize()

        drag_force_magnitude = -2 * current_forward_speed
        self.wheel.ApplyForce(drag_force_magnitude * current_forward_normal, self.wheel.worldCenter, True)


class Car():

    def __init__(self, box2world, position=None):

        if position is None:
            position = [POS_X + 10, POS_Y + 5]
        
        self.destroy_flag = False
        self.position = position
        self.ticking = 0
        self.box2world = box2world
        self.body = self.box2world.CreateDynamicBody(position=self.position,
                                                     angle=0.0)
        self.body_Fixtures = self.body.CreatePolygonFixture(shape=b2PolygonShape(box=(2.189 / 2, 4.897 / 2)),
                                                            density=2810,
                                                            friction=0,
                                                            filter=b2Filter(
                                                                categoryBits=CAR_CATEGORY,
                                                                maskBits=PEDESTRIAN_CATEGORY + BUILDING_CATEGORY,
                                                                groupIndex=CAR_GROUP))
        self.tires = []
        self.suspensions = []
        self.on_the_beginning = False
        self.body.userData = {'obj': self}

        # Connecting bodywork with wheels

        for i in range(4):
            if i == 1 or i == 2:  # Moving wheels a bit away from the chassis
                X_SHIFT = 0.45
            else:
                X_SHIFT = -0.45

            # Creating Wheels
            self.tires.append(Wheel(box2world=box2world, phi=0, position=(self.position[0] + X_SHIFT, self.position[1])))

            if i in range(2):
                # Rear wheels do not move so they can be welded to chassis
                self.suspensions.append(box2world.CreateWeldJoint(bodyA=self.body,
                                                                  bodyB=self.tires[i].wheel,
                                                                  localAnchorA=(self.body_Fixtures.shape.vertices[i][0] + X_SHIFT, self.body_Fixtures.shape.vertices[i][1] + 0.5),
                                                                  localAnchorB=(0, 0),
                                                                  ))
            else:
                # Front wheels are revolute joints
                self.suspensions.append(box2world.CreateRevoluteJoint(bodyA=self.tires[i].wheel,
                                                                      bodyB=self.body,
                                                                      localAnchorA=(0, 0),
                                                                      localAnchorB=(self.body_Fixtures.shape.vertices[i][0] + X_SHIFT, self.body_Fixtures.shape.vertices[i][1] - 0.5),
                                                                      enableMotor=True,
                                                                      maxMotorTorque=100000,
                                                                      enableLimit=True,
                                                                      lowerAngle=0,
                                                                      upperAngle=0,
                                                                      motorSpeed=0
                                                                      ))

    def control(self):  # This makes control independent from visualisation
        # Front left suspension: index 2
        # Front right suspension: index 3

        # Alternative steering (Con: Physical perversion, Pro: It works...)
        if keyboard.is_pressed("a"):
            self.suspensions[2].lowerLimit -= np.deg2rad(2)
            self.suspensions[2].upperLimit -= np.deg2rad(2)
            self.suspensions[3].lowerLimit -= np.deg2rad(2)
            self.suspensions[3].upperLimit -= np.deg2rad(2)

            if self.suspensions[2].lowerLimit <= -np.pi / 4:

                self.suspensions[2].upperLimit = -np.pi / 4
                self.suspensions[2].lowerLimit = -np.pi / 4
                self.suspensions[3].upperLimit = -np.pi / 4
                self.suspensions[3].lowerLimit = -np.pi / 4

        if keyboard.is_pressed("d"):
            self.suspensions[2].upperLimit += np.deg2rad(2)
            self.suspensions[2].lowerLimit += np.deg2rad(2)
            self.suspensions[3].upperLimit += np.deg2rad(2)
            self.suspensions[3].lowerLimit += np.deg2rad(2)

            if self.suspensions[2].upperLimit >= np.pi / 4:
                self.suspensions[2].lowerLimit = np.pi / 4
                self.suspensions[2].upperLimit = np.pi / 4
                self.suspensions[3].lowerLimit = np.pi / 4
                self.suspensions[3].upperLimit = np.pi / 4
            else:
                self.ticking = -1

        FORWARD_IMPULSE = 1000
        if keyboard.is_pressed("w"):
            self.tires[2].wheel.ApplyLinearImpulse(b2Vec2(self.tires[2].wheel.GetWorldVector((0, FORWARD_IMPULSE))), self.tires[2].wheel.position, True)
            self.tires[3].wheel.ApplyLinearImpulse(b2Vec2(self.tires[3].wheel.GetWorldVector((0, FORWARD_IMPULSE))), self.tires[3].wheel.position, True)

        if keyboard.is_pressed("s"):

            self.tires[2].wheel.ApplyLinearImpulse(b2Vec2(self.tires[2].wheel.GetWorldVector((0, -FORWARD_IMPULSE))), self.tires[2].wheel.position, True)
            self.tires[3].wheel.ApplyLinearImpulse(b2Vec2(self.tires[3].wheel.GetWorldVector((0, -FORWARD_IMPULSE))), self.tires[3].wheel.position, True)

        for wheels in self.tires:
            if wheels == 0 or wheels == 1:
                pass
            else:
                Wheel.apply_impulse(wheels)

    def destroy_when_out_of_map(self):
        if self.body.position[0] < -10:
            self.destroy_flag = True
        elif self.body.position[0] >= 120:
            self.destroy_flag = True
        elif self.body.position[1] < 0:
            self.destroy_flag = True
        elif self.body.position[1] >= 60:
            self.destroy_flag = True

def print_useful_stuff(car, pedestrian, *number):
    """
    This function is used to print any information needed such as pedestrian or vehicle velocity, 
    distance of given vehicle from the nearest pedestrian and so on and so forth...
    """

    LIMIT = 10
    difference = np.sqrt((pedestrian.body.position[0] - car.body.position[0])**2 + (pedestrian.body.position[1] - car.body.position[1])**2)

    car_velocity = car.body.__GetLinearVelocity()
    car_position = car.body.position
    
    if difference <= LIMIT and car_velocity != (0,0):
        if number:
            print(f"Pozor, chodec c. {number} je presne: {difference} daleko")
        else:
            print(f"Pozor, chodec je presne: {difference} daleko")
        

class Obstacle():
    def __init__(self, box2world, position, shape_choice):
        self.shape_choice = shape_choice
        self.position = position
        self.box2world = box2world
        self.nearest_building = 0
        self.body = self.box2world.CreateStaticBody(position=position,
                                                     angle=0.0,
                                                     fixtures=b2FixtureDef(
                                                     shape=b2CircleShape(radius=random.uniform(1,2)) if shape_choice == 0 else b2PolygonShape(box=(2,2)),
                                                     density=2,
                                                     friction=0.3,
                                                     filter=b2Filter(
                                                             categoryBits=PEDESTRIAN_CATEGORY,
                                                             maskBits= CAR_CATEGORY + BUILDING_CATEGORY,
                                                             groupIndex=PEDESTRIAN_GROUP)))
        self.current_position = [self.body.position]
        self.body.userData = {'obj': self}


class Pedestrian():
    def __init__(self, box2world, ped_velocity=25, position=None,):

        if position is None:
            position = [5, 5]
        self.destroy_flag = False
        self.ped_velocity = ped_velocity
        self.position = position
        self.box2world = box2world
        self.nearest_building = 0
        self.body = self.box2world.CreateDynamicBody(position=position,
                                                     angle=0.0,
                                                     fixtures=b2FixtureDef(
                                                         shape=b2CircleShape(radius=1),
                                                         density=2,
                                                         friction=0.3,
                                                         filter=b2Filter(
                                                             categoryBits=PEDESTRIAN_CATEGORY,
                                                             maskBits= CAR_CATEGORY + BUILDING_CATEGORY,
                                                             groupIndex=PEDESTRIAN_GROUP)))
        self.current_position = [self.body.position]
        self.body.userData = {'obj': self}


def pedestrian_walk(pedestrian, buildings):
    # Find the building nearest to the pedestrian
    list_of_distances = []

    for building in buildings:
        list_of_distances.append(np.sqrt((pedestrian.body.position[0] - building.footprint.position[0])**2 + (pedestrian.body.position[1] - building.footprint.position[1])**2))
    pedestrian.nearest_building = list_of_distances.index(min(list_of_distances))

    building = buildings[pedestrian.nearest_building]

    # Pedestrian walks around either the left or right side of its nearest building
    if building.position[0] - 1.075 * building.shape[0] <= pedestrian.body.position[0] <= building.position[0] + 1.075 * building.shape[0] and building.position[1] - building.shape[1] <= pedestrian.position[1] <= building.position[1] + 1.075 * building.shape[1]:
        pedestrian.body.__SetLinearVelocity(b2Vec2(0, pedestrian.ped_velocity))
        pedestrian.current_position.append(pedestrian.body.position)
        if pedestrian.current_position[-2] == pedestrian.current_position[-1]:
            pedestrian.body.__SetLinearVelocity(b2Vec2(pedestrian.ped_velocity, 0))

    # Pedestrian walks around either lower or upper side of its nearest building
    elif building.position[1] - 1.075 * building.shape[1] <= pedestrian.body.position[1] <= building.position[1] + 1.075 * building.shape[1]:
        pedestrian.body.__SetLinearVelocity(b2Vec2(-pedestrian.ped_velocity, 0))
        pedestrian.current_position.append(pedestrian.body.position)
        if pedestrian.current_position[-2] == pedestrian.current_position[-1]:
            pedestrian.body.__SetLinearVelocity(b2Vec2(0, pedestrian.ped_velocity))
    if len(pedestrian.current_position) == 15:
        pedestrian.current_position = [pedestrian.body.position]
    # print(self.current_position)

    # Make pedestrian-wraparound effect
    if pedestrian.body.position[1] > 100:
        pedestrian.body.position = pedestrian.body.position - (0, 170)
    elif pedestrian.body.position[1] <= -70:
        pedestrian.body.position = pedestrian.body.position + (0, 170)
    elif pedestrian.body.position[0] >= 302:
        pedestrian.body.position = pedestrian.body.position - (300, 0)
    elif pedestrian.body.position[0] <= -2:
        pedestrian.body.position = pedestrian.body.position + (300, 0)


class Building():

    BUILDING_FILTER = b2Filter(categoryBits=BUILDING_CATEGORY,
                               maskBits=PEDESTRIAN_CATEGORY + CAR_CATEGORY,
                               groupIndex=BUILDING_GROUP,
                               )

    def __init__(self, box2world, shape, position, sensor=None):
        self.box2world = box2world
        self.destroy_flag = False
        self.shape = shape
        self.position = position

        if sensor is None:
            sensor = False

        self.sensor = sensor
        self.footprint = self.box2world.CreateStaticBody(position=position,
                                                         angle=0.0,
                                                         fixtures=b2FixtureDef(
                                                             shape=b2PolygonShape(box=(self.shape)),
                                                             density=1000,
                                                             friction=1000,
                                                             filter=Building.BUILDING_FILTER))
        self.footprint.userData = {'obj': self}


############################################################## Setting up car and buildings

def create_buildings():
    BUILDINGS_POSITIONS = [(POS_X - 25, POS_Y), (POS_X + 40, POS_Y), (POS_X - 25, POS_Y + 40), (POS_X + 40, POS_Y + 40),
                       (POS_X - 25, POS_Y - 40), (POS_X + 40, POS_Y - 40), (POS_X - 25, POS_Y - 80), (POS_X + 40, POS_Y + -80),
                       (POS_X + 165, POS_Y), (POS_X + 105, POS_Y), (POS_X + 165, POS_Y + 40), (POS_X + 105, POS_Y + 40),
                       (POS_X + 165, POS_Y - 40), (POS_X + 105, POS_Y - 40), (POS_X + 165, POS_Y - 80), (POS_X + 105, POS_Y + -80),
                       (POS_X + 230, POS_Y + 40), (POS_X + 230, POS_Y), (POS_X + 230, POS_Y - 40), (POS_X + 230, POS_Y + - 80)]

    # Generate buildings of different shape (the number of buildings is partly determined by an element of coincidence)
    RANDOM_MODULO = random.randint(3,7)
    for i in range(len(BUILDINGS_POSITIONS)):
        if i % RANDOM_MODULO == 0:
            skyscrapers.append(Building(box2world, shape=(8, 12), position=(BUILDINGS_POSITIONS[i][0] + 15.5, BUILDINGS_POSITIONS[i][1])))
            skyscrapers.append(Building(box2world, shape=(10, 12), position=(BUILDINGS_POSITIONS[i][0] - 15.5, BUILDINGS_POSITIONS[i][1])))

        else:
            skyscrapers.append(Building(box2world, shape=(23, 12), position=(BUILDINGS_POSITIONS[i])))
skyscrapers = []
create_buildings()

def create_pedestrians(MAX_AMOUNT_PEDESTRIANS=20):
    k = 0
    skyscraper_to_walk_around = []
    positive_negative = []
    while k <= MAX_AMOUNT_PEDESTRIANS:
        random_skyscraper = random.choice(skyscrapers)  # Randomly choose a skyscraper next to which a pedestrian is generated
        skyscraper_to_walk_around.append(random_skyscraper)
        new_walker = Pedestrian(box2world, position=(random_skyscraper.position[0] - random.randint(-random_skyscraper.shape[0], random_skyscraper.shape[0]),
                                random_skyscraper.position[1] - random.randint(-random_skyscraper.shape[1], random_skyscraper.shape[1])))
        johnnie_walkers.append(new_walker)
        positive_negative.append([-1, 1][random.randrange(2)])
        k = k + 1
johnnie_walkers = []
create_pedestrians()


# Randomly choose a skyscraper, next to which a car will be generated
# (first four skyscraper are visible when the PyGame window is launched)
cars_random_skyscraper = random.choice(skyscrapers[0:3])
def create_car():
    cars.append(Car(box2world, position=(cars_random_skyscraper.position[0] + 1.55 * cars_random_skyscraper.shape[0], cars_random_skyscraper.position[1])))

cars = []
create_car()

############################################################## Pygame visualisation
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), HWSURFACE | DOUBLEBUF | RESIZABLE)
pygame.display.set_caption('Top Down Car Using OOP')
colors = {dynamicBody: (133, 187, 101, 0), staticBody: (15, 0, 89, 0)}
running = True
EVERY_SECOND = 0
FPS = 24
TIME_STEP = 1.0 / FPS
k = 1


############################### Drawing functions

SCREEN_OFFSETX, SCREEN_OFFSETY = SCREEN_WIDTH / 16, SCREEN_HEIGHT
def fix_vertices(vertices):
    return [(int(SCREEN_OFFSETX + v[0]), int(SCREEN_OFFSETY - v[1])) for v in vertices]


def _draw_polygon(polygon, screen, body, fixture):
    transform = body.transform
    vertices = fix_vertices([transform * v * PPM for v in polygon.vertices])
    pygame.draw.polygon(
        screen, [c / 2.0 for c in colors[body.type]], vertices, 0)
    pygame.draw.polygon(screen, colors[body.type], vertices, 1)
polygonShape.draw = _draw_polygon


def _draw_circle(circle, screen, body, fixture):
    position = fix_vertices([body.transform * circle.pos * PPM])[0]
    pygame.draw.circle(screen, colors[body.type],
                       position, int(circle.radius * PPM))
circleShape.draw = _draw_circle


def _draw_edge(edge, screen, body, fixture):
    vertices = fix_vertices(
        [body.transform * edge.vertex1 * PPM, body.transform * edge.vertex2 * PPM])
    pygame.draw.line(screen, colors[body.type], vertices[0], vertices[1])
edgeShape.draw = _draw_edge



def step():
    if (cars[0].destroy_flag == False):
        TIME_STEP = 1.0 / FPS
        box2world.Step(TIME_STEP, 10, 10)
    else:
        for body in box2world.bodies:
            box2world.DestroyBody(body)

############################### Main game loop
while running:
    # Control your car
    cars[0].control()
    cars[0].destroy_when_out_of_map()
    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            running = False
        # Zoom In/Out
        elif event.type == KEYDOWN and event.key == pygame.K_KP_PLUS:
            PPM += 5
        elif event.type == KEYDOWN and event. key == pygame.K_KP_MINUS:
            if PPM <= 5:
                PPM -= 0
            else:
                PPM -= 5


    # Draw the world
    screen.fill((255, 255, 255, 255))
    if event.type == VIDEORESIZE:
        screen = pygame.display.set_mode(event.dict['size'], HWSURFACE | DOUBLEBUF | RESIZABLE)

    for body in box2world.bodies:
        for fixture in body.fixtures:
            fixture.shape.draw(screen, body, fixture)
  
    for aragorn in range(len(johnnie_walkers)):
        pedestrian_walk(johnnie_walkers[aragorn], skyscrapers)
    print_useful_stuff(cars[0], johnnie_walkers[0], 0)
    
    # Simulate dynamic equation in each step
    step()
    _reset()

    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()  # Update the full display Surface to the screen
    pygame.time.Clock().tick(FPS)
    EVERY_SECOND += 1


pygame.quit()
print('Done!')
