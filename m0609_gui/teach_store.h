// Taught points for the M0609 GUI: ~/m0609_points_<robot serial>.json (one file per robot).
//   joints [deg], posx [mm, ZYZ deg] (base, as reported by the controller), sol = solution space (0..7) used by
//   movejx so an approach point is reached with the same arm configuration as the taught pose.
#pragma once
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QString>

#include <array>
#include <map>

struct TeachPoint {
    std::array<float, 6> joints{}, posx{};
    int sol = -1;    // solution space, -1 = unknown
    QString grip;    // gripper state when saved (info only)
    QString time;
};
using PointMap = std::map<QString, TeachPoint>;

inline QString points_path(const QString& serial) {
    return QDir::homePath() + "/m0609_points_" + (serial.isEmpty() ? QString("unknown") : serial) + ".json";
}

inline PointMap load_points(const QString& serial) {
    PointMap m;
    QFile f(points_path(serial));
    if (!f.open(QIODevice::ReadOnly)) return m;
    const QJsonObject pts = QJsonDocument::fromJson(f.readAll()).object().value("points").toObject();
    for (auto it = pts.begin(); it != pts.end(); ++it) {
        const QJsonObject o = it.value().toObject();
        const QJsonArray j = o.value("joints").toArray(), x = o.value("posx").toArray();
        if (j.size() != 6 || x.size() != 6) continue;
        TeachPoint p;
        for (int i = 0; i < 6; ++i) {
            p.joints[i] = static_cast<float>(j[i].toDouble());
            p.posx[i] = static_cast<float>(x[i].toDouble());
        }
        p.sol = o.value("sol").toInt(-1);
        p.grip = o.value("gripper").toString();
        p.time = o.value("time").toString();
        m[it.key()] = p;
    }
    return m;
}

inline bool save_points(const QString& serial, const PointMap& m, QString* err) {
    QJsonObject pts;
    for (const auto& [name, p] : m) {
        QJsonArray j, x;
        for (int i = 0; i < 6; ++i) {
            j.append(std::round(p.joints[i] * 1e4) / 1e4);
            x.append(std::round(p.posx[i] * 1e4) / 1e4);
        }
        pts[name] = QJsonObject{{"joints", j}, {"posx", x}, {"sol", p.sol}, {"gripper", p.grip}, {"time", p.time}};
    }
    const QJsonObject root{{"version", 1}, {"robot", "M0609"}, {"serial", serial}, {"points", pts}};
    QSaveFile f(points_path(serial));  // atomic: never leaves a half-written file
    if (!f.open(QIODevice::WriteOnly) || f.write(QJsonDocument(root).toJson()) < 0 || !f.commit()) {
        if (err) *err = f.errorString();
        return false;
    }
    return true;
}
