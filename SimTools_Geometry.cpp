// ============================================================
// SimTools v2.0 - 几何计算模块实现
// ============================================================

#include "SimTools_v2.h"
#include <cmath>
#include <algorithm>

namespace SimTools {

    // ============================================================
    // 点与多边形关系
    // ============================================================

    bool Geometry::IsPointInTriangle(const Point2D& point,
                                    const Point2D& a,
                                    const Point2D& b,
                                    const Point2D& c) {
        // 使用叉积（向量积）判断点是否在三角形内
        // 计算点相对于三角形三条边的位置

        double sign1 = (b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x);
        double sign2 = (c.x - b.x) * (point.y - b.y) - (c.y - b.y) * (point.x - b.x);
        double sign3 = (a.x - c.x) * (point.y - c.y) - (a.y - c.y) * (point.x - c.x);

        // 检查是否在同侧（都为正或都为负）
        bool has_neg = (sign1 < 0) || (sign2 < 0) || (sign3 < 0);
        bool has_pos = (sign1 > 0) || (sign2 > 0) || (sign3 > 0);

        return !(has_neg && has_pos);
    }

    bool Geometry::IsPointInPolygon(const Point2D& point,
                                   const std::vector<Point2D>& polygon) {
        // 射线法：从点向右发射水平射线，统计与多边形边相交的次数
        // 奇数次相交：在内部，偶数次相交：在外部

        if (polygon.size() < 3) {
            return false;
        }

        int intersections = 0;
        int n = static_cast<int>(polygon.size());

        for (int i = 0, j = n - 1; i < n; j = i++) {
            // 检查边 (polygon[j], polygon[i])
            double yi = polygon[i].y;
            double yj = polygon[j].y;
            double xi = polygon[i].x;
            double xj = polygon[j].x;

            // 检查点是否在边的Y范围内
            if ((yi > point.y) != (yj > point.y)) {
                // 计算射线与边的交点的X坐标
                double intersect_x = (xj - xi) * (point.y - yi) / (yj - yi) + xi;

                // 如果交点在点的右侧，则计数
                if (point.x < intersect_x) {
                    intersections++;
                }
            }
        }

        return (intersections % 2) == 1;
    }

    // ============================================================
    // 距离计算
    // ============================================================

    double Geometry::DistanceToLineSegment(const Point2D& point,
                                          const Point2D& line_start,
                                          const Point2D& line_end) {
        // 计算点到线段的最近距离
        double dx = line_end.x - line_start.x;
        double dy = line_end.y - line_start.y;
        double length_squared = dx * dx + dy * dy;

        // 如果线段退化为点
        if (length_squared < 1e-10) {
            return Distance(point, line_start);
        }

        // 计算投影参数 t
        double t = ((point.x - line_start.x) * dx + (point.y - line_start.y) * dy) / length_squared;

        // 限制 t 在 [0, 1] 范围内
        t = std::max(0.0, std::min(1.0, t));

        // 计算投影点
        Point2D projection(
            line_start.x + t * dx,
            line_start.y + t * dy
        );

        return Distance(point, projection);
    }

    // ============================================================
    // 三维几何
    // ============================================================

    Vector3d Geometry::PlaneNormal(const Vector3d& p1,
                                         const Vector3d& p2,
                                         const Vector3d& p3) {
        // 计算两个向量
        Vector3d v1 = p2 - p1;
        Vector3d v2 = p3 - p1;

        // 叉积得到法向量
        Vector3d normal = v1.cross(v2);

        // 归一化
        double norm = normal.norm();
        if (norm > 1e-10) {
            normal /= norm;
        }

        return normal;
    }

    double Geometry::DistanceToPlane(const Vector3d& point,
                                    const Vector3d& plane_point,
                                    const Vector3d& plane_normal) {
        // 点到平面的有向距离：((point - plane_point) · normal)
        Vector3d diff = point - plane_point;
        double distance = diff.dot(plane_normal);
        return std::abs(distance);
    }

} // namespace SimTools
