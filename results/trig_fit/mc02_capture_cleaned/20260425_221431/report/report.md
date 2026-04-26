# 三角基函数重力拟合报告

- 输入 CSV: `E:\RoboMaster\mec_arm\gravity_identification\data\mc02_capture_cleaned.csv`
- 样本数量: `10720`
- 测试集抽样步长: `5`

## Joint2

### Basis Terms
- `1`
- `sin(q2)`
- `cos(q2)`
- `sin(q3)`
- `cos(q3)`
- `sin(q2+q3)`
- `cos(q2+q3)`
- `sin(q2+q3+q4)`
- `cos(q2+q3+q4)`
- `sin(q2+q3+q4+q5)`
- `cos(q2+q3+q4+q5)`

### Train Metrics
- MAE: `0.549574`
- RMSE: `0.660958`
- MaxAbs: `2.205619`
- Corr: `0.9631772878579705`

### Test Metrics
- MAE: `0.546830`
- RMSE: `0.660754`
- MaxAbs: `2.168000`
- MAE / mean(|tau|): `0.09234322171878376`
- Corr: `0.9633732815292415`

## Joint3

### Basis Terms
- `1`
- `sin(q2)`
- `cos(q2)`
- `sin(q3)`
- `cos(q3)`
- `sin(q2+q3)`
- `cos(q2+q3)`
- `sin(q2+q3+q4)`
- `cos(q2+q3+q4)`
- `sin(q2+q3+q4+q5)`
- `cos(q2+q3+q4+q5)`

### Train Metrics
- MAE: `0.522622`
- RMSE: `0.635541`
- MaxAbs: `2.462171`
- Corr: `0.9046564554171445`

### Test Metrics
- MAE: `0.522149`
- RMSE: `0.630254`
- MaxAbs: `2.159694`
- MAE / mean(|tau|): `0.07747159320848059`
- Corr: `0.9064822421577707`
