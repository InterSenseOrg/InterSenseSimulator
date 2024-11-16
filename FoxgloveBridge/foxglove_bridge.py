import asyncio
import sys
import time
import struct
import numpy as np
from base64 import b64encode
from traceback import print_exception
from typing import Set, Type
import sim_messages_pb2
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import ChannelId

try:
    from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage as FgCompressedImage
    from foxglove_schemas_protobuf.PointCloud_pb2 import PointCloud as FgPointCloud
    from foxglove_schemas_protobuf.PackedElementField_pb2 import PackedElementField as FgPackedElementField
    from foxglove_schemas_protobuf.SceneEntity_pb2 import SceneEntity as FgSceneEntity
    from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate as FgSceneUpdate
    from foxglove_schemas_protobuf.Pose_pb2 import Pose as FgPose
    from foxglove_schemas_protobuf.CubePrimitive_pb2 import CubePrimitive as FgCubePrimitive 
    from foxglove_schemas_protobuf.Vector3_pb2 import Vector3 as FgVector3
    from foxglove_schemas_protobuf.Point2_pb2 import Point2 as FgPoint2
    from foxglove_schemas_protobuf.Color_pb2 import Color as FgColor
    from foxglove_schemas_protobuf.ImageAnnotations_pb2 import ImageAnnotations as FgImageAnnotations
    from foxglove_schemas_protobuf.PointsAnnotation_pb2 import PointsAnnotation  as FgPointsAnnotation 

    import google.protobuf.message
    from google.protobuf.descriptor_pb2 import FileDescriptorSet
    from google.protobuf.descriptor import FileDescriptor
except ImportError as err:
    print_exception(*sys.exc_info())
    print(
        "Unable to import protobuf schemas. Run `pip install 'foxglove-websocket[examples]'`?",
    )
    sys.exit(1)


SIM_IP = '127.0.0.1'
SIM_PORT = 5030
FOXGLOVE_PORT = 8765


def build_file_descriptor_set(
    message_class: Type[google.protobuf.message.Message],
) -> FileDescriptorSet:
    """
    Build a FileDescriptorSet representing the message class and its dependencies.
    """
    file_descriptor_set = FileDescriptorSet()
    seen_dependencies: Set[str] = set()

    def append_file_descriptor(file_descriptor: FileDescriptor):
        for dep in file_descriptor.dependencies:
            if dep.name not in seen_dependencies:
                seen_dependencies.add(dep.name)
                append_file_descriptor(dep)
        file_descriptor.CopyToProto(file_descriptor_set.file.add())  # type: ignore

    append_file_descriptor(message_class.DESCRIPTOR.file)
    return file_descriptor_set


channel_id_dict = {}

async def get_channel_id(server, channel_name, msg_type):
    global channel_id_dict

    if channel_name in channel_id_dict.keys():
        channel_id = channel_id_dict[channel_name]
    else:

        channel_id = await server.add_channel(
            {
                "topic": channel_name,
                "encoding": "protobuf",
                "schemaName": msg_type.DESCRIPTOR.full_name,
                "schema": b64encode(
                    build_file_descriptor_set(msg_type).SerializeToString()
                ).decode("ascii"),
                "schemaEncoding": "protobuf",
            }
        )

        channel_id_dict[channel_name] = channel_id

    return channel_id


async def on_compressed_image(msg: sim_messages_pb2.CompressedImage, server):
    global channel_id_dict
    channel_name = f'/camera/{msg.header.frame_id}'
    channel_id = await get_channel_id(server, channel_name, FgCompressedImage)

    fg_msg = FgCompressedImage()
    fg_msg.timestamp.FromNanoseconds(int(msg.header.timestamp*1e3))
    fg_msg.frame_id = msg.header.frame_id
    fg_msg.data = msg.data
    fg_msg.format = msg.format
    
    now = time.time_ns()

    await server.send_message(channel_id, now, fg_msg.SerializeToString())

async def on_point_cloud(msg: sim_messages_pb2.PointCloud, server):
    global channel_id_dict
    channel_name = f'/lidar/{msg.header.frame_id}'
    channel_id = await get_channel_id(server, channel_name, FgPointCloud)

    pose = FgPose()

    points = []

    for point in msg.point:
        points.append([point.x, point.y, point.z])   
        
    points_np = np.array(points, dtype=np.float32)
    points_flat = points_np.flatten()

    points_binary = points_flat.tobytes()

    fg_msg_pc = FgPointCloud()
    fg_msg_pc.timestamp.FromNanoseconds(int(msg.header.timestamp*1e3))
    fg_msg_pc.frame_id = msg.header.frame_id
    
    now = time.time_ns()

    fg_msg_pc.data = points_binary

    # Set the point cloud fields (like "x", "y", "z" and their datatype)
    fg_msg_pc.fields.extend([
        FgPackedElementField(name="x", offset=0, type=FgPackedElementField.NumericType.FLOAT32),
        FgPackedElementField(name="y", offset=4, type=FgPackedElementField.NumericType.FLOAT32),
        FgPackedElementField(name="z", offset=8, type=FgPackedElementField.NumericType.FLOAT32)
    ])

    fg_msg_pc.point_stride = 12  # 4 bytes each for x, y, z -> 12 bytes total
    fg_msg_pc.pose.position.x = pose.position.x
    fg_msg_pc.pose.position.y = pose.position.y
    fg_msg_pc.pose.position.z = pose.position.z
    fg_msg_pc.pose.orientation.x = pose.orientation.x
    fg_msg_pc.pose.orientation.y = pose.orientation.y
    fg_msg_pc.pose.orientation.z = pose.orientation.z
    fg_msg_pc.pose.orientation.w = 1

    await server.send_message(channel_id, now, fg_msg_pc.SerializeToString())

async def on_scene_update(msg: sim_messages_pb2.Detection3DArray, server):
    global channel_id_dict
    channel_name = f'/GT3D/{msg.header.frame_id}'
    channel_id = await get_channel_id(server, channel_name, FgSceneUpdate)

    pose = FgPose()
    size = FgVector3()
    color = FgColor(r = 1, g = 0, b = 0, a = 0.5)

    detections = []

    for detection in msg.detections:
        pose.position.x = detection.local_position.x
        pose.position.y = detection.local_position.y
        pose.position.z = detection.local_position.z
        pose.orientation.x = detection.local_rotation.qx
        pose.orientation.y = detection.local_rotation.qy
        pose.orientation.z = detection.local_rotation.qz
        pose.orientation.w = detection.local_rotation.qw
        size.x = detection.length
        size.y = detection.width
        size.z = detection.height
        detections.append(FgCubePrimitive(pose = pose, size = size, color = color)) 
        
    fg_msg_su = FgSceneUpdate()
    fg_msg = FgSceneEntity()
    fg_msg.timestamp.FromNanoseconds(int(msg.header.timestamp*1e3))
    fg_msg.frame_id = msg.header.frame_id
    fg_msg.id = "1"
    fg_msg.cubes.extend(detections)
    fg_msg_su.entities.append(fg_msg)
    
    now = time.time_ns()

    await server.send_message(channel_id, now, fg_msg_su.SerializeToString())


async def on_image_annotations(msg: sim_messages_pb2.Detection2DArray, server):
    global channel_id_dict
    channel_name = f'/GT2D/{msg.header.frame_id}'
    channel_id = await get_channel_id(server, channel_name, FgImageAnnotations)

    color = FgColor(r = 1, g = 0, b = 0, a = 0.5)

    point = FgPoint2()
    points = []
    
    for detection in msg.detections:
        point.x = detection.bbox.xmax - detection.bbox.xmin
        points.append(point)

    fg_msg_ia = FgImageAnnotations()
    fg_msg_points = FgPointsAnnotation()
    fg_msg_points.timestamp.FromNanoseconds(int(msg.header.timestamp*1e3))
    fg_msg_points.type = FgPointsAnnotation.Type.LINE_STRIP
    fg_msg_points.points.extend(points)
    fg_msg_ia.points.append(fg_msg_points)

    now = time.time_ns()
    await server.send_message(channel_id, now, fg_msg_ia.SerializeToString())


async def receive_message(sim_reader):
    # Read message length first (assuming 4 bytes length prefix)
    raw_msglen = await sim_reader.read(4)
    if not raw_msglen:
        return None

    # Unpack the 32-bit integer length from the byte data
    msglen = struct.unpack('<i', raw_msglen)[0]

    data = b''
    while len(data) < msglen:
        packet = await sim_reader.read(msglen - len(data))
        if not packet:
            return None
        data += packet

    # Read the message data according to the length
    return data


async def main():


    class Listener(FoxgloveServerListener):
        async def on_subscribe(self, server: FoxgloveServer, channel_id: ChannelId):
            print("First client subscribed to", channel_id)

        async def on_unsubscribe(self, server: FoxgloveServer, channel_id: ChannelId):
            print("Last client unsubscribed from", channel_id)


    sim_reader, _ = await asyncio.open_connection(SIM_IP, SIM_PORT)

    async with FoxgloveServer("127.0.0.1", FOXGLOVE_PORT, "example server") as server:
        server.set_listener(Listener())


        while True:
            data = await receive_message(sim_reader)

            if not data:
                print("No data received, connection may have been closed")
                break

            common_message = sim_messages_pb2.CommonMessage()
            common_message.ParseFromString(data)

            if common_message.HasField('CompressedImage'):
                await on_compressed_image(common_message.CompressedImage, server)
            elif common_message.HasField('PointCloud'):
                await on_point_cloud(common_message.PointCloud, server)
            elif common_message.HasField('Detection3DArray'):
                await on_scene_update(common_message.Detection3DArray, server)       
            elif common_message.HasField('Detection2DArray'):
                await on_image_annotations(common_message.Detection2DArray, server)                    
            else:
                print(f"WARNING: Received message without parsing: {common_message}")

            

if __name__ == "__main__":
    run_cancellable(main())