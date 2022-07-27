
import random
import math
import sys

def expo_gen(lam):
    return -1 / lam * math.log(1 - random.random())


def generate_routefile(run_time, lam1, lam2, fix_seed):
    if fix_seed:
        random.seed(22)  # make tests reproducible
    else:
        seed = random.randrange(sys.maxsize)
        random.seed(seed)

    print("Seed was:", seed)

    N = run_time  # number of time steps
    # demand per second from different directions
    lam13 = lam1
    lam42 = lam2

    next_13 = expo_gen(lam13)
    next_42 = expo_gen(lam42)

    with open("tlc_single_straight.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="v1" accel="2.6" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="55" \
guiShape="passenger"/>

        <route id="r13" edges="e10 e03" />
        <route id="r42" edges="e40 e02" />""", file=routes)
        vehNr = 0
        for i in range(N):
            if i > next_13:
                print('    <vehicle id="right_%i" type="v1" route="r13" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                next_13 += expo_gen(lam13)

            if i > next_42:
                print('    <vehicle id="up_%i" type="v1" route="r42" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                next_42 += expo_gen(lam42)

        print("</routes>", file=routes)


def generate_routefile_Veberod(file_name, run_time, lam1, lam2, fix_seed):
    seed = 3570926575676058649
    if not fix_seed:
        seed = random.randrange(sys.maxsize)
    random.seed(seed)

    print("Seed was:", seed)

    N = run_time  # number of time steps
    # demand per second from different directions
    lam13 = lam1
    lam42 = lam2

    next_13 = expo_gen(lam13)
    next_42 = expo_gen(lam42)

    with open(file_name, "w") as routes:
        print("""<routes>
        <vType id="v1" accel="2.6" decel="4.5" sigma="0.5" length="5" minGap="1" maxSpeed="55" \
guiShape="passenger"/>

        <route id="r13" edges="96027913#0 96027913#2 30186293 311389510#1" />
        <route id="r42" edges="-355113043#1 -24626686#2 -24626686#1 -34992983#3 -34992983#1" />""", file=routes)
        vehNr = 0
        for i in range(N):
            if i > next_13:
                print('    <vehicle id="right_%i" type="v1" route="r13" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                next_13 += expo_gen(lam13)

            if i > next_42:
                print('    <vehicle id="up_%i" type="v1" route="r42" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                next_42 += expo_gen(lam42)

        print("</routes>", file=routes)


def generate_routefile_pedestrian(file_name, run_time, lam1, lam2, fix_seed):

    seed = 3570926575676058649
    if not fix_seed:
        seed = random.randrange(sys.maxsize)
    random.seed(seed)

    print("Seed was:", seed)

    N = run_time  # number of time steps
    # demand per second from different directions
    lam_w31 = lam1
    lam_w42 = lam2

    next_w31 = expo_gen(lam_w31)
    next_w42 = expo_gen(lam_w42)

    with open(file_name, "w") as routes:
        print('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">', file=routes)

        pedNr = 0
        for i in range(N):
            if i > next_w31:
                print("""   <person id="ped_%i" depart="%i" departPos="random">
        <personTrip from="34992983#2" to="24626686#2" arrivalPos="random"/>
    </person>""" % (pedNr, i), file=routes)
                pedNr += 1
                next_w31 += expo_gen(lam_w31)

            if i > next_w42:
                print("""
    <person id="ped_%i" depart="%i" departPos="random">
        <personTrip from="96027913#2" to="30186293" arrivalPos="random"/>
    </person>""" % (pedNr, i), file=routes)
                pedNr += 1
                next_w42 += expo_gen(lam_w42)

        print("</routes>", file=routes)
