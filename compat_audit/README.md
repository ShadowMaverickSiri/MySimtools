# SimTools 旧版/新版兼容性审计

本目录保存两版输出探针和数值比较脚本。旧版探针链接原始 `SimTools.cpp`，新版探针链接当前静态库；两者使用同一批输入。

矩阵专项覆盖三轴9组角度和全球7×7经纬度网格；当前审计共比较1278个旧版/新版数值项。

## 已核对的对应关系

| 旧版 | 新版 | 结论 |
|---|---|---|
| `BoundNorm2`、`Normalize`、角度规整 | `Math::*` | 有效输入一致；新版补充零向量保护 |
| `Interp2`、`Interp_L7`、`Insert_EL` | `Interpolation::Linear/Lagrange7/LagrangeGlobal` | 有效区间一致；新版修复旧版上边界越界 |
| `RangKutta` | `Numerical::RungeKutta4` | 自治方程逐值一致；新版修复旧版多步积分未推进时间的问题 |
| `Gps2E`、`E2Gps`、`E2Gps_Newton` | `Coordinate::GpsToEcef/EcefToGps/EcefToGpsNewton` | 常规点一致且新版精度更高；补充地轴/地心处理 |
| `CoordinateTransM3` | `Coordinate::RotationMatrix` | 数值一致 |
| `MakeE2NFromGps/MakeN2EFromGps` | `EcefToNueMatrix/NueToEcefMatrix` | 数值一致，坐标顺序明确为北天东 |
| `VnFromV` | `Coordinate::VelocityToNue` | 数值一致；已修复新版曾发生的 NUE/NED 语义错位 |
| `VeFromV` | `NueToEcefVelocity([V,0,0])` | 允许旧版低精度 π 常量误差后一致 |
| `BigCircle` | `GreatCircleDistance` | 数值一致 |
| `Vincenty_inverse/Vincenty` | `VincentyInverse/VincentyDirect` | 距离、起点方位角、终点坐标一致；新版统一返回终点前向方位角 |
| `GPS_R_Compute` | `SiteDistance` | 允许旧版椭球近似误差后一致 |
| `PhiL_Compute` | `SiteAzimuth` | 旧版返回东向为负的带符号角；新版采用从北顺时针 `[0,360)` 标准方位角 |
| `AirParaFromH` | `Atmosphere::GetParameters` | 已恢复位势高度换算，标准分层输出一致 |
| `Rand_N(mu,sigma2)` | `Random::NormalFromVariance` | 新增显式方差兼容接口 |
| `Ifpointraingle` | `Geometry::IsPointInTriangle` | 数值一致 |
| `Angle60To10/Angle10To60` | `DmsToDecimal/DecimalToDms` | 数值一致 |
| `MatMul/MatTrans/MatMul33` | `MatrixUtils` 同类接口 | 数值一致 |
| `TxtRowcount` | `FileIO::CountLines` | 普通数据文件一致；新版对空行和注释行采用有效数据行语义 |
| `Index1`、`NumberToString` | `FindIndex`、`FileIO::ToString` | 常规输入一致；新版边界行为更安全 |

## 有意不逐值复制的旧行为

- `Caculate_g` 是旧版经验线性重力公式，而 `AirParaFromH` 又使用另一套反平方公式。新版统一使用标准大气中的反平方重力模型。
- `SoundSpeedFromH` 和 `RhoFromH` 与旧版 `AirParaFromH` 在部分高度层互相矛盾；新版统一以分层标准大气参数为准。
- 旧版 `Rand_N` 使用 12 个均匀随机数近似正态分布，新版使用标准库正态分布，因此随机序列不逐样本相同，只核对分布参数语义。
- 旧版 Vincenty 的 `A21` 在正解和反解中定义不一致；新版统一为终点前向方位角。反方位角应使用 `Regulate360(azimuth2 + 180)`。
- 旧版 `Sitecompute_func` 的第二参数是垂直平面内的倾角，新版 `TargetFromSite` 的第二参数是标准方位角，二者不是同一接口。新版通过求解仰角，同时严格满足方位角、目标高度和斜距约束。
- 旧版 `Interp3` 实际执行局部二维拉格朗日插值且包含累加位置错误；新版 `Bilinear` 是明确的双线性插值接口，不逐值复制该旧行为。

比较脚本会跳过上述定义明确变化的输出，其余结果按各物理量的合理精度阈值检查。
