"""Panda3D helpers: plain meshes, polygons with holes and line sets to GeomNodes."""
import math

from panda3d.core import (Geom, GeomNode, GeomTriangles, GeomVertexData, GeomVertexFormat, GeomVertexWriter, LineSegs,
                          Triangulator, Vec4)


def mesh_node(mesh, name=None):
    """A vehicle_mesh.Mesh (vertices, per-vertex colours, triangles) as a GeomNode with smooth normals."""
    vdata = GeomVertexData(name or mesh.name, GeomVertexFormat.getV3n3c4(), Geom.UHStatic)
    vdata.setNumRows(len(mesh.v))
    vw, nw, cw = GeomVertexWriter(vdata, "vertex"), GeomVertexWriter(vdata, "normal"), GeomVertexWriter(vdata, "color")
    for p, n, c in zip(mesh.v, mesh.normals(), mesh.c):
        vw.addData3(*p)
        nw.addData3(*n)
        cw.addData4(*c)
    tris = GeomTriangles(Geom.UHStatic)
    for a, b, c in mesh.t:
        tris.addVertices(a, b, c)
    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode(name or mesh.name)
    node.addGeom(geom)
    return node


class Builder:
    """Accumulates flat-shaded triangles (position, normal, colour) into one GeomNode."""

    def __init__(self, name):
        self.name = name
        self.vdata = GeomVertexData(name, GeomVertexFormat.getV3n3c4(), Geom.UHStatic)
        self.vw = GeomVertexWriter(self.vdata, "vertex")
        self.nw = GeomVertexWriter(self.vdata, "normal")
        self.cw = GeomVertexWriter(self.vdata, "color")
        self.tris = GeomTriangles(Geom.UHStatic)
        self.n = 0

    def tri(self, a, b, c, color, normal=None):
        if normal is None:
            ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            wx, wy, wz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
            nx, ny, nz = uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx
            L = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            normal = (nx / L, ny / L, nz / L)
        for p in (a, b, c):
            self.vw.addData3(*p)
            self.nw.addData3(*normal)
            self.cw.addData4(*color)
        self.tris.addVertices(self.n, self.n + 1, self.n + 2)
        self.n += 3

    def quad(self, a, b, c, d, color):
        self.tri(a, b, c, color)
        self.tri(a, c, d, color)

    def box(self, cx, cy, z0, sx, sy, sz, yaw, color, top=None):
        """An upright box: centre (cx, cy), base at z0, heading yaw; `top` colours the roof face."""
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = 0.5 * sx, 0.5 * sy
        pts = [(cx + c * x - s * y, cy + s * x + c * y) for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
        z1 = z0 + sz
        for i in range(4):
            (ax, ay), (bx, by) = pts[i], pts[(i + 1) % 4]
            self.quad((ax, ay, z0), (bx, by, z0), (bx, by, z1), (ax, ay, z1), color)
        self.quad(*[(x, y, z1) for x, y in pts], top or color)

    def polygon(self, outer, z, color, holes=()):
        """A horizontal polygon with holes (Panda3D's triangulator), facing up."""
        tri = Triangulator()
        for x, y in outer:
            tri.addPolygonVertex(tri.addVertex(x, y))
        for h in holes:
            tri.beginHole()
            for x, y in h:
                tri.addHoleVertex(tri.addVertex(x, y))
        tri.triangulate()
        for i in range(tri.getNumTriangles()):
            pa, pb, pc = (tri.getVertex(tri.getTriangleV0(i)), tri.getVertex(tri.getTriangleV1(i)), tri.getVertex(tri.getTriangleV2(i)))
            a, b, c = (pa[0], pa[1], z), (pb[0], pb[1], z), (pc[0], pc[1], z)
            area = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
            if area < 0:
                b, c = c, b
            self.tri(a, b, c, color, (0.0, 0.0, 1.0))

    def node(self):
        geom = Geom(self.vdata)
        geom.addPrimitive(self.tris)
        n = GeomNode(self.name)
        n.addGeom(geom)
        return n


def lines(name, polylines, color, thickness=2.0, z=0.05):
    ls = LineSegs(name)
    ls.setThickness(thickness)
    ls.setColor(Vec4(*color))
    for pl in polylines:
        if len(pl) < 2:
            continue
        x, y = pl[0][0], pl[0][1]
        ls.moveTo(x, y, pl[0][2] if len(pl[0]) > 2 else z)
        for p in pl[1:]:
            ls.drawTo(p[0], p[1], p[2] if len(p) > 2 else z)
    return ls.create()
