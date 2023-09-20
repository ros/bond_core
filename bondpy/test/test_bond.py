import rclpy
from rclpy.duration import Duration

from bondpy.bondpy import Bond


def test_connection():
    context = rclpy.Context()
    rclpy.init(context=context)
    try:
        node_1 = rclpy.create_node('test_node_1', context=context)
        node_2 = rclpy.create_node('test_node_2', context=context)

        bond_a = Bond(node_1, 'test_topic_bond', 'test_id')
        bond_b = Bond(node_2, 'test_topic_bond', 'test_id')

        bond_a.start()
        bond_b.start()

        assert bond_a.wait_until_formed(timeout=Duration(seconds=30))
        assert bond_b.wait_until_formed(timeout=Duration(seconds=30))

    finally:
        rclpy.shutdown(context=context)


def test_disconnection():
    context = rclpy.Context()
    rclpy.init(context=context)
    try:
        node_1 = rclpy.create_node('test_node_1', context=context)
        node_2 = rclpy.create_node('test_node_2', context=context)

        bond_a = Bond(node_1, 'test_topic_bond', 'test_id')
        bond_b = Bond(node_2, 'test_topic_bond', 'test_id')

        bond_a.start()
        bond_b.start()

        bond_a.wait_until_formed()
        bond_a.break_bond()

        assert bond_a.wait_until_broken(timeout=Duration(seconds=30))
        assert bond_b.wait_until_broken(timeout=Duration(seconds=30))

    finally:
        rclpy.shutdown(context=context)
