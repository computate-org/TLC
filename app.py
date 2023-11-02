from flask import Flask
from threading import Event
from consumer import FlaskKafka
from kafka import  KafkaProducer
from sumolib import checkBinary  # noqa
from kazoo.client import KazooClient
from kazoo.exceptions import NodeExistsError
from lxml import etree
import sumolib
import json
import signal
import os
import sys
import traceback
import main_pedestrian
import pyproj

app = Flask(__name__)

INTERRUPT_EVENT = Event()

kafka_brokers = os.environ.get('KAFKA_BROKERS') or "kafka0.apps-crc.testing:32000"
kafka_group_run = os.environ.get('KAFKA_GROUP_RUN') or "smartvillage-kafka-run"
kafka_group_stop = os.environ.get('KAFKA_GROUP_STOP') or "smartvillage-kafka-stop"
kafka_topic_sumo_run = os.environ.get('KAFKA_TOPIC_SUMO_RUN') or "smartvillage-sumo-run"
kafka_topic_sumo_run_report = os.environ.get('KAFKA_TOPIC_SUMO_RUN_REPORT') or "smartvillage-sumo-run-report"
kafka_topic_sumo_stop = os.environ.get('KAFKA_TOPIC_SUMO_STOP') or "smartvillage-sumo-stop"
kafka_topic_simulation_info = os.environ.get('KAFKA_TOPIC_SIMULATION_INFO') or "smartvillage-sumo-simulation-info"
kafka_topic_simulation_info_patch = os.environ.get('KAFKA_TOPIC_SIMULATION_INFO_PATCH') or "smartvillage-sumo-simulation-info-patch"
kafka_topic_location_info = os.environ.get('KAFKA_TOPIC_TRAFFIC_FLOW_OBSERVED_INFO') or "smartvillage-sumo-traffic-flow-observed-info"
kafka_topic_location_info_patch = os.environ.get('KAFKA_TOPIC_TRAFFIC_FLOW_OBSERVED_INFO_PATCH') or "smartvillage-sumo-traffic-flow-observed-info-patch"
kafka_security_protocol = os.environ.get('KAFKA_SECURITY_PROTOCOL') or "SSL"
# Run: oc extract -n smartvillage secret/smartvillage-kafka-cluster-ca-cert --to=/opt/kafka/truststore/ --keys=ca.crt --confirm
kafka_ssl_cafile = os.environ.get('KAFKA_SSL_CAFILE') or "/opt/kafka/truststore/ca.crt"
# Run: oc extract -n smartvillage secret/smartvillage-kafka --to=/opt/kafka/keystore/ --keys=user.crt --confirm
kafka_ssl_certfile = os.environ.get('KAFKA_SSL_CERTFILE') or "/opt/kafka/keystore/user.crt"
# Run: oc extract -n smartvillage secret/smartvillage-kafka --to=/opt/kafka/keystore/ --keys=user.key --confirm
kafka_ssl_keyfile = os.environ.get('KAFKA_SSL_KEYFILE') or "/opt/kafka/keystore/user.key"
kafka_max_poll_records = int(os.environ.get('KAFKA_MAX_POLL_RECORDS') or "1")
kafka_max_poll_interval_ms = int(os.environ.get('KAFKA_MAX_POLL_INTERVAL_MS') or "3000000")

zookeeper_host_name = os.environ.get('ZOOKEEPER_HOST_NAME') or "zookeeper.apps-crc.testing"
zookeeper_port = int(os.environ.get('ZOOKEEPER_PORT') or "30081")

bus_run = FlaskKafka(INTERRUPT_EVENT
         , bootstrap_servers=",".join([kafka_brokers])
         , group_id=kafka_group_run
         , security_protocol=kafka_security_protocol
         , ssl_cafile=kafka_ssl_cafile
         , ssl_certfile=kafka_ssl_certfile
         , ssl_keyfile=kafka_ssl_keyfile
#             , max_poll_interval_ms=kafka_max_poll_interval_ms
         , max_poll_records=kafka_max_poll_records
         )

bus_stop = FlaskKafka(INTERRUPT_EVENT
         , bootstrap_servers=",".join([kafka_brokers])
         , group_id=kafka_group_stop
         , security_protocol=kafka_security_protocol
         , ssl_cafile=kafka_ssl_cafile
         , ssl_certfile=kafka_ssl_certfile
         , ssl_keyfile=kafka_ssl_keyfile
#             , max_poll_interval_ms=kafka_max_poll_interval_ms
         , max_poll_records=kafka_max_poll_records
         )

producer = KafkaProducer(
        bootstrap_servers=kafka_brokers
        , security_protocol=kafka_security_protocol
        , ssl_cafile=kafka_ssl_cafile
        , ssl_certfile=kafka_ssl_certfile
        , ssl_keyfile=kafka_ssl_keyfile
        )

@bus_run.handle(kafka_topic_sumo_run)
def kafka_topic_sumo_run_handle(msg):
    try:
        bus_run.consumer.commit()
        print("received message from %s topic: %s" % (kafka_topic_sumo_run, msg))
        sumoBinary = checkBinary('sumo')
        simulation_report = json.loads(msg.value)

        zk = KazooClient(hosts='%s:%s' % (zookeeper_host_name, zookeeper_port))
        zk.start()
        try:
            zk.create("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"), bytes("Running", 'utf-8'), makepath=True)
        except NodeExistsError as e:
            zk.set("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"), bytes("Running", 'utf-8'))
            pass

        initial_par = simulation_report.get('paramInitialPar', [10., 20., 30., 50., 10., 10., 8., 8., 5., 5.])
        initial_par = [float(s) for s in initial_par]

        lam = simulation_report.get('paramLam', [10., 10., 6., 6.])
        lam = [float(s) for s in lam]

        demand_scale = simulation_report.get('paramDemandScale', [1., 1.])
        step_size = float(simulation_report.get('paramStepSize', 1.))
        run_time = int(simulation_report.get('paramRunTime', 1000))
        total_iter_num = int(simulation_report.get('paramTotalIterNum', 10))
        iters_per_par = int(simulation_report.get('paramItersPerPar', 5))

        result = { "pk": simulation_report.get("pk"), "setUpdatedParameters": [], "setUpdatedPerformance": [], "setReportStatus": "Running", "setReportProgress": "0" }
        producer.send(kafka_topic_sumo_run_report, json.dumps(result).encode('utf-8'))
    
        updated_parameters, updated_performance = main_pedestrian.ipa_gradient_method_pedestrian(
                zk
                , producer
                , simulation_report
                , initial_par=initial_par
                , lam=lam
                , demand_scale=demand_scale
                , step_size=step_size
                , run_time=run_time
                , total_iter_num=total_iter_num
                , iters_per_par=iters_per_par
                , print_mode=False)

        if len(updated_parameters[0]) == total_iter_num:
            result = { "pk": simulation_report.get("pk"), "setUpdatedParameters": updated_parameters, "setUpdatedPerformance": updated_performance, "setReportStatus": "Completed", "setReportProgress": "100" }
        else:
            result = { "pk": simulation_report.get("pk"), "setUpdatedParameters": updated_parameters, "setUpdatedPerformance": updated_performance, "setReportStatus": "Stopped" }
        producer.send(kafka_topic_sumo_run_report, json.dumps(result).encode('utf-8'))
        zk.set("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"), bytes("Completed", 'utf-8'))

        zk.stop()
    except Exception as e:
        ex_type, ex_value, ex_traceback = sys.exc_info()
        # Extract unformatter stack traces as tuples
        trace_back = traceback.extract_tb(ex_traceback)
        result = { "pk": simulation_report.get("pk"), "setReportStatus": "Error" }
        producer.send(kafka_topic_sumo_run_report, json.dumps(result).encode('utf-8'))
        zk.set("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"), bytes("Error", 'utf-8'))

        if zk:
            zk.stop()
    
        # Format stacktrace
        stack_trace = list()
    
        for trace in trace_back:
            stack_trace.append("File : %s , Line : %d, Func.Name : %s, Message : %s" % (trace[0], trace[1], trace[2], trace[3]))
    
        print("%s occured processing a message on %s topic: %s\n%s" % (ex_type.__name__, kafka_topic_sumo_run, ex_value, '\n'.join(stack_trace)))

@bus_stop.handle(kafka_topic_sumo_stop)
def kafka_topic_sumo_stop_handle(msg):
    try:
        simulation_report = json.loads(msg.value)

        zk = KazooClient(hosts='%s:%s' % (zookeeper_host_name, zookeeper_port))
        zk.start()
        (zookeeper_report_status, node_stat) = zk.get("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"))
        print("zookeeper_report_status: %s" % zookeeper_report_status)
        try:
            zk.create("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"), bytes("Stop", 'utf-8'), makepath=True)
        except NodeExistsError as e:
            zk.set("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"), bytes("Stop", 'utf-8'))
        (zookeeper_report_status, node_stat) = zk.get("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"))
        print("zookeeper_report_status: %s" % zookeeper_report_status)
        print("received STOP message from %s topic: %s %s" % (kafka_topic_sumo_run, simulation_report.get("pk"), simulation_report.get("objectTitle")))

        zk.stop()
    except Exception as e:
        ex_type, ex_value, ex_traceback = sys.exc_info()
        # Extract unformatter stack traces as tuples
        trace_back = traceback.extract_tb(ex_traceback)

        zk.set("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"), bytes("Error", 'utf-8'))

        if zk:
            zk.stop()
    
        # Format stacktrace
        stack_trace = list()
    
        for trace in trace_back:
            stack_trace.append("File : %s , Line : %d, Func.Name : %s, Message : %s" % (trace[0], trace[1], trace[2], trace[3]))
    
        print("%s occured processing a message on %s topic: %s\n%s" % (ex_type.__name__, kafka_topic_sumo_run, ex_value, '\n'.join(stack_trace)))

@bus_stop.handle(kafka_topic_simulation_info)
def kafka_topic_sumo_simulation_info(msg):
    try:
        traffic_simulation = json.loads(msg.value)
        pk_str = traffic_simulation.get('pk')

        print("Recieved TrafficSimulation %s info message from %s topic" % (pk_str, kafka_topic_simulation_info))

        lane_area_detector_ids = []
        lane_area_detector_lanes = []
        lane_area_detector_paths = []

        e1_detector_ids = []
        e1_detector_lanes = []
        e1_detector_paths = []

        traffic_simulation = json.loads(msg.value)
        sumocfg_path = traffic_simulation.get('sumocfgPath')
        if sumocfg_path:
            sumocfg = etree.parse(sumocfg_path)
            net_file_paths = [os.path.abspath(os.path.join(sumocfg_path, '..', path)) for path in sumocfg.xpath('//configuration/input/net-file/@value')]
            net_file_path = net_file_paths[0]
            additional_file_paths = [os.path.abspath(os.path.join(sumocfg_path, '..', path)) for path in sumocfg.xpath('//configuration/input/additional-files/@value')]
            route_file_path_strings = [os.path.abspath(os.path.join(sumocfg_path, '..', path)) for path in sumocfg.xpath('//configuration/input/route-files/@value')]
            route_file_paths = []
            if route_file_path_strings:
                for route_file_path_string in route_file_path_strings:
                    for route_file_path in route_file_path_string.split(','):
                        route_file_paths.append(route_file_path.strip())

            net_file = etree.parse(net_file_path)
            net = sumolib.net.readNet(net_file_path)

            walking_area_ids = []
            walking_area_lanes = []
            walkingarea_edges = net_file.xpath('//net/edge[@function="walkingarea"]')
            for walkingarea_edge in walkingarea_edges:
                walking_area_lane_id = walkingarea_edge.xpath('@id')[0]
                walking_area_ids.append(walking_area_lane_id)
                coordinates = []
                walking_area_lane_data = {"id": walking_area_lane_id, "type": "Polygon", "coordinates": coordinates}
                walking_area_lanes.append(walking_area_lane_data)
                walkingarea_shapes = walkingarea_edge.xpath('lane/@shape')
                for walkingarea_shape in walkingarea_shapes:
                    walkingarea_points = walkingarea_shape.split(" ")
                    for walkingarea_point in walkingarea_points:
                        walkingarea_point_parts = walkingarea_point.split(',')
                        coords = net.convertXY2LonLat(float(walkingarea_point_parts[0]), float(walkingarea_point_parts[1]))
                        coordinates.append([float(coords[0]), float(coords[1])])

            for additional_file_path in additional_file_paths:
                additional_file = etree.parse(additional_file_path)

                lane_area_detectors = additional_file.xpath('//additional/laneAreaDetector')
                for lane_area_detector in lane_area_detectors:
                    lane_area_detector_ids.append(lane_area_detector.xpath('@id')[0])
                    lane_area_detector_paths.append(os.path.abspath(os.path.join(additional_file_path, '..', lane_area_detector.xpath('@file')[0])))


                    lane_area_detector_lane_string = lane_area_detector.xpath('@lanes')[0]
                    lane_area_detector_lane = []
                    lane_area_detector_lanes.append(lane_area_detector_lane)
                    for lane_area_detector_lane_id in lane_area_detector_lane_string.split(" "):
                        coordinates = []
                        lane_area_detector_lane_data = {"id": lane_area_detector_lane_id, "type": "LineString", "coordinates": coordinates}
                        lane_area_detector_lane.append(lane_area_detector_lane_data)
                        edge_points = net_file.xpath('//net/edge/lane[@id="%s"]/@shape' % lane_area_detector_lane_id)[0].split(" ")
                        for edge_point in edge_points:
                            edge_point_parts = edge_point.split(',')
                            coords = net.convertXY2LonLat(float(edge_point_parts[0]), float(edge_point_parts[1]))
                            coordinates.append([float(coords[0]), float(coords[1])])

                e1_detectors = additional_file.xpath('//additional/e1Detector')
                for e1_detector in e1_detectors:
                    e1_detector_ids.append(e1_detector.xpath('@id')[0])
                    e1_detector_lanes.append(e1_detector.xpath('@lane')[0])
                    e1_detector_paths.append(os.path.abspath(os.path.join(additional_file_path, '..', e1_detector.xpath('@file')[0])))

            patch_body = {
                "pk": pk_str

                , "setLaneAreaDetectorIds": lane_area_detector_ids
                , "setLaneAreaDetectorLanes": lane_area_detector_lanes
                , "setLaneAreaDetectorPaths": lane_area_detector_paths

                , "setE1DetectorIds": e1_detector_ids
                , "setE1DetectorLanes": e1_detector_lanes
                , "setE1DetectorPaths": e1_detector_paths

                , "setWalkingAreaIds": walking_area_ids
                , "setWalkingAreaLanes": walking_area_lanes
            }
            producer.send(kafka_topic_simulation_info_patch, json.dumps(patch_body).encode('utf-8'))
            print("Sent PATCH TrafficSimulation %s message to %s topic" % (pk_str, kafka_topic_simulation_info_patch))
    except Exception as e:
        ex_type, ex_value, ex_traceback = sys.exc_info()
        # Extract unformatter stack traces as tuples
        trace_back = traceback.extract_tb(ex_traceback)
        print("%s occured processing a message on %s topic: %s\n%s" % (ex_type.__name__, kafka_topic_simulation_info, ex_value, '\n'.join(trace_back)))

@bus_stop.handle(kafka_topic_location_info)
def kafka_topic_sumo_location_info(msg):
    try:
        traffic_flow_observed = json.loads(msg.value)
        # pk_str = traffic_flow_observed.get('pk')
        # class_simple_name = traffic_flow_observed.get('classSimpleName')
        #
        # print("Recieved %s %s info message from %s topic" % (class_simple_name, pk_str, kafka_topic_location_info))
        #
        # lane_area_detector_ids = []
        # lane_area_detector_lanes = []
        # lane_area_detector_paths = []
        #
        # e1_detector_ids = []
        # e1_detector_lanes = []
        # e1_detector_paths = []
        #
        # traffic_flow_observed = json.loads(msg.value)
        # sumocfg_path = traffic_flow_observed.get('sumocfgPath')
        # if sumocfg_path:
        #     sumocfg = etree.parse(sumocfg_path)
        #     net_file_paths = [os.path.abspath(os.path.join(sumocfg_path, '..', path)) for path in sumocfg.xpath('//configuration/input/net-file/@value')]
        #     additional_file_paths = [os.path.abspath(os.path.join(sumocfg_path, '..', path)) for path in sumocfg.xpath('//configuration/input/additional-files/@value')]
        #
        #     for additional_file_path in additional_file_paths:
        #         additional_file = etree.parse(additional_file_path)
        #
        #         lane_area_detectors = additional_file.xpath('//additional/laneAreaDetector')
        #         for lane_area_detector in lane_area_detectors:
        #             lane_area_detector_ids.append(lane_area_detector.xpath('@id')[0])
        #             lane_area_detector_lanes.append(lane_area_detector.xpath('@lanes')[0])
        #             lane_area_detector_paths.append(os.path.abspath(os.path.join(additional_file_path, '..', lane_area_detector.xpath('@file')[0])))
        #
        #         e1_detectors = additional_file.xpath('//additional/e1Detector')
        #         for e1_detector in e1_detectors:
        #             #print(etree.tostring(e1_detector, pretty_print=True))
        #             e1_detector_ids.append(e1_detector.xpath('@id')[0])
        #             e1_detector_lanes.append(e1_detector.xpath('@lane')[0])
        #             e1_detector_paths.append(os.path.abspath(os.path.join(additional_file_path, '..', e1_detector.xpath('@file')[0])))
        #
        #     patch_body = {
        #         "pk": pk_str
        #
        #         , "setLaneAreaDetectorIds": lane_area_detector_ids
        #         , "setLaneAreaDetectorLanes": lane_area_detector_lanes
        #         , "setLaneAreaDetectorPaths": lane_area_detector_paths
        #
        #         , "setE1DetectorIds": e1_detector_ids
        #         , "setE1DetectorLanes": e1_detector_lanes
        #         , "setE1DetectorPaths": e1_detector_paths
        #     }
        #     producer.send(kafka_topic_simulation_info_patch, json.dumps(patch_body).encode('utf-8'))
        #     print("Sent PATCH TrafficSimulation %s message to %s topic" % (pk_str, kafka_topic_simulation_info_patch))
    except Exception as e:
        ex_type, ex_value, ex_traceback = sys.exc_info()
        # Extract unformatter stack traces as tuples
        trace_back = traceback.extract_tb(ex_traceback)
        print("%s occured processing a message on %s topic: %s\n%s" % (ex_type.__name__, kafka_topic_location_info, ex_value, '\n'.join(trace_back)))
    # try:
    #     net = sumolib.net.readNet('/home/ctate/.local/src/TLC/input/Veberod_intersection_pedestrian.net.xml')
    #     edge_shapes = [
    #         "175.13,734.43 177.69,752.95 180.82,774.94 183.17,784.52"
    #         , "187.08,795.35 189.01,799.98 196.82,813.25 208.02,832.13"
    #         , "208.17,832.40 214.93,844.44 216.53,846.30"
    #         ]
    #     for edge_shape in edge_shapes:
    #         edge_points = edge_shape.split(' ')
    #         lat = []
    #         lon = []
    #         for edge_point in edge_points:
    #             edge_point_parts = edge_point.split(',')
    #             coords = net.convertXY2LonLat(float(edge_point_parts[0]), float(edge_point_parts[1]))
    #             lat.append(float(coords[1]))
    #             lon.append(float(coords[0]))
    #         print()
    #         print("lat: %s" % lat)
    #         print("lon: %s" % lon)
    # except Exception as e:
    #     ex_type, ex_value, ex_traceback = sys.exc_info()
    #     # Extract unformatter stack traces as tuples
    #     trace_back = traceback.extract_tb(ex_traceback)
    #     print("%s occured processing a message on %s topic: %s\n%s" % (ex_type.__name__, kafka_topic_location_info, ex_value, '\n'.join(trace_back)))

def listen_kill_server():

    signal.signal(signal.SIGTERM, bus_run.interrupted_process)
    signal.signal(signal.SIGINT, bus_run.interrupted_process)
    signal.signal(signal.SIGQUIT, bus_run.interrupted_process)
    signal.signal(signal.SIGHUP, bus_run.interrupted_process)

    signal.signal(signal.SIGTERM, bus_stop.interrupted_process)
    signal.signal(signal.SIGINT, bus_stop.interrupted_process)
    signal.signal(signal.SIGQUIT, bus_stop.interrupted_process)
    signal.signal(signal.SIGHUP, bus_stop.interrupted_process)

@app.route('/')
def hello():
    return "Hello World!"

def start():
    flask_port = os.environ.get('FLASK_PORT') or 8081
    flask_port = int(flask_port)

    bus_run.run()
    bus_stop.run()
    listen_kill_server()
    app.run(port=flask_port, host='0.0.0.0')

if __name__ == "__main__":
    start()

