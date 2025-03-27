#!/usr/bin/env python

from __future__ import print_function
# from binascii import hexlify
import asn1tools
import paho.mqtt.publish as publish
# import paho.mqtt.client as mqtt
import time
# import json

# hello_world = asn1tools.compile_string(SPECIFICATION, 'uper')
topic = "v2x"
HOST = "39.106.250.159"
PORT = 1883
user="admin"
passwd = "public"

asn = asn1tools.compile_files(['BSM.asn','MsgFrame.asn','DefTime.asn','DefPosition.asn','VehStatus.asn',
                                        'DefMotion.asn','DefAcceleration.asn','VehBrake.asn','VehSize.asn','VehClass.asn',
                                        'VehSafetyExt.asn','DefPositionOffset.asn','VehEmgExt.asn','Map.asn','MapNode.asn',
                                        'MapLink.asn','MapSpeedLimit.asn','MapLane.asn','MapPoint.asn','SPATIntersectionState.asn',
                                        'RSM.asn','SignalPhaseAndTiming.asn','RSI.asn'
                                        ], numeric_enums=True)


# print(asn.types)



_spat= {
        "msgCnt": 0,
        "intersections": 
            [
            {'intersectionId': {"id":100},
             'status': (b'manualControlIsEnabled', 16*11),#length
             "phases": 
             [
                {"id":1,"phaseStates":[{"light":0}]}

             ]
            }
            # ,
            # {'intersectionId': {"id":101},
            #  'status': (b'manualControlIsEnabled', 0),
            #  "phases":
            #  [
            #     {"id":1,"phaseStates":[{"light":'flashing-green'}]}

            #  ]
            # }
            ]
        }

encoded = asn.encode("MessageFrame", (
                            'spatFrame', _spat
                        ) 
                    )


client_ID = time.strftime('%Y%m%d%H%M%S',time.localtime(time.time()))
publish.single(topic , encoded, qos = 1,hostname=HOST,port=PORT,
                    client_id=client_ID,auth = { 'username': user,
                                                      'password': passwd})
# publish.single(topic , json.dumps(_json), qos = 1,hostname=HOST,port=PORT,
#                     client_id=client_ID,auth = { 'username': user,
#                                                       'password': passwd})
time.sleep(4)

# print('Encoded:', hexlify(encoded).decode('ascii'))
print( asn.decode('MessageFrame', encoded))
print( asn.decode('MessageFrame', encoded)[1]["intersections"][0]["status"][0].decode('utf-8'))
