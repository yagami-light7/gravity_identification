# 三角基函数重力拟合报告

- 输入 CSV: `E:\RoboMaster\mec_arm\gravity_identification\data\mc02_capture_cleaned.csv`
- 样本数量: `10593`
- 测试集抽样步长: `5`
- Ridge 正则系数: `0.01`

## Joint1

### Basis Terms
- `1`
- `sin(q1)`
- `cos(q1)`

### Train Metrics
- MAE: `0.290557`
- RMSE: `0.358852`
- MaxAbs: `1.472551`
- Corr: `0.07879010574134471`

### Test Metrics
- MAE: `0.291329`
- RMSE: `0.359333`
- MaxAbs: `1.411016`
- MAE / mean(|tau|): `0.9773035030185654`
- Corr: `0.07441504933038785`

## Joint2

### Basis Terms
- `1`
- `sin(q2)`
- `cos(q2)`
- `sin(q2+q3)`
- `cos(q2+q3)`
- `sin(q2+q3+q4)`
- `cos(q2+q3+q4)`
- `sin(q2+q3+q4+q5)`
- `cos(q2+q3+q4+q5)`

### Train Metrics
- MAE: `0.695461`
- RMSE: `0.974303`
- MaxAbs: `9.003242`
- Corr: `0.9861791263636907`

### Test Metrics
- MAE: `0.698247`
- RMSE: `0.979239`
- MaxAbs: `8.628980`
- MAE / mean(|tau|): `0.12488179942094911`
- Corr: `0.9860468215858345`

## Joint3

### Basis Terms
- `1`
- `sin(q2+q3)`
- `cos(q2+q3)`
- `sin(q2+q3+q4)`
- `cos(q2+q3+q4)`
- `sin(q2+q3+q4+q5)`
- `cos(q2+q3+q4+q5)`

### Train Metrics
- MAE: `0.516945`
- RMSE: `0.682951`
- MaxAbs: `3.567798`
- Corr: `0.9711016650628429`

### Test Metrics
- MAE: `0.516018`
- RMSE: `0.680541`
- MaxAbs: `2.632934`
- MAE / mean(|tau|): `0.08884510498343812`
- Corr: `0.9712776310246019`

## Joint4

### Basis Terms
- `1`
- `sin(q4)`
- `cos(q4)`
- `sin(q2+q3+q4)`
- `cos(q2+q3+q4)`
- `sin(q2+q3+q4+q5)`
- `cos(q2+q3+q4+q5)`

### Train Metrics
- MAE: `0.023136`
- RMSE: `0.029706`
- MaxAbs: `0.282943`
- Corr: `0.6962939573000396`

### Test Metrics
- MAE: `0.023257`
- RMSE: `0.030097`
- MaxAbs: `0.280685`
- MAE / mean(|tau|): `0.5374271270040074`
- Corr: `0.6901783448879727`

## Joint5

### Basis Terms
- `1`
- `sin(q5)`
- `cos(q5)`
- `sin(q2+q3+q4+q5)`
- `cos(q2+q3+q4+q5)`

### Train Metrics
- MAE: `0.231007`
- RMSE: `0.315304`
- MaxAbs: `1.063678`
- Corr: `0.7733527836297329`

### Test Metrics
- MAE: `0.231676`
- RMSE: `0.316960`
- MaxAbs: `1.025664`
- MAE / mean(|tau|): `0.28651028287988173`
- Corr: `0.771615761211122`

## Joint6

### Basis Terms
- `1`
- `sin(q6)`
- `cos(q6)`

### Train Metrics
- MAE: `0.778088`
- RMSE: `1.070855`
- MaxAbs: `3.765751`
- Corr: `0.336066445286585`

### Test Metrics
- MAE: `0.778005`
- RMSE: `1.073530`
- MaxAbs: `3.981709`
- MAE / mean(|tau|): `0.9290407067029982`
- Corr: `0.333705024626077`
