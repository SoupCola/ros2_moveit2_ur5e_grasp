#!/usr/bin/env python3
# Copyright 2026 Antigravity
import sys
import json
from typing import List, Dict, Any, Optional

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    sys.stderr.write("Error: rclpy not found. Source your ROS 2 environment first.\n")
    sys.exit(1)

try:
    from mcp.server.fastmcp import FastMCP
except ImportError:
    sys.stderr.write("Error: 'mcp' python package not found. Please install it with: pip install mcp\n")
    sys.exit(1)


class ROS2GraphScanner:
    """Encapsulates interaction with the ROS 2 Daemon to query graph info."""

    def __init__(self, node_name: str = '_agent_context_scanner'):
        self.node_name = node_name

    def scan(self) -> Dict[str, Any]:
        """Initializes a temp node, queries graph, and shuts down."""
        rclpy.init(args=None)
        node = rclpy.create_node(self.node_name)
        
        try:
            return self._query_node_api(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def _query_node_api(self, node: Node) -> Dict[str, Any]:
        """Internal method to extract data from the node API."""
        node_names = node.get_node_names_and_namespaces()
        topic_names_and_types = node.get_topic_names_and_types()
        service_names_and_types = node.get_service_names_and_types()
        
        return {
            "nodes": [
                {"name": name, "namespace": ns, "full_name": f"{ns}/{name}".replace("//", "/")}
                for name, ns in node_names
            ],
            "topics": [
                {"name": name, "types": types}
                for name, types in topic_names_and_types
            ],
            "services": [
                {"name": name, "types": types}
                for name, types in service_names_and_types
            ]
        }


class ROS2ContextServer:
    """Wraps the FastMCP server and exposes ROS 2 capabilities."""

    def __init__(self, server_name: str = "ros2_context_provider"):
        self.server = FastMCP(server_name)
        self.scanner = ROS2GraphScanner()
        
        # Register resources and tools
        self.server.resource("ros2://graph")(self.get_graph)
        self.server.tool()(self.list_nodes)
        self.server.tool()(self.list_topics)

    def get_graph(self) -> str:
        """Resource: Returns the current ROS 2 graph (nodes, topics, services)."""
        try:
            data = self.scanner.scan()
            return json.dumps(data, indent=2)
        except Exception as e:
            return json.dumps({"error": str(e)})

    def list_nodes(self) -> str:
        """Tool: Lists all active ROS 2 nodes."""
        graph = self.scanner.scan()
        return json.dumps(graph["nodes"], indent=2)

    def list_topics(self) -> str:
        """Tool: Lists all active ROS 2 topics."""
        graph = self.scanner.scan()
        return json.dumps(graph["topics"], indent=2)

    def run(self):
        """Starts the MCP server."""
        self.server.run()


if __name__ == "__main__":
    app = ROS2ContextServer()
    app.run()
