# 三角基函数重力拟合报告

- 输入 CSV: `E:\RoboMaster\mec_arm\gravity_identification\data\mc02_capture_cleaned.csv`
- 样本数量: `10660`
- 测试集抽样步长: `5`
- Ridge 正则系数: `0.01`

## Joint1

### Basis Terms
- `1`
- `sin(q1)`
- `cos(q1)`

### Train Metrics
- MAE: `0.249625`
- RMSE: `0.304191`
- MaxAbs: `0.980304`
- Corr: `0.25132862575898723`

### Test Metrics
- MAE: `0.248233`
- RMSE: `0.303400`
- MaxAbs: `0.973062`
- MAE / mean(|tau|): `0.8773827906181678`
- Corr: `0.25353177671857785`

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
- MAE: `0.897991`
- RMSE: `1.079657`
- MaxAbs: `3.635560`
- Corr: `0.9867261180076042`

### Test Metrics
- MAE: `0.894029`
- RMSE: `1.076333`
- MaxAbs: `3.405362`
- MAE / mean(|tau|): `0.10863543428758297`
- Corr: `0.9867804521668126`

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
- MAE: `0.670957`
- RMSE: `0.775683`
- MaxAbs: `2.030939`
- Corr: `0.9492206099368752`

### Test Metrics
- MAE: `0.667921`
- RMSE: `0.772566`
- MaxAbs: `2.121303`
- MAE / mean(|tau|): `0.11538444693941063`
- Corr: `0.9497390992561379`

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
- MAE: `0.017080`
- RMSE: `0.025122`
- MaxAbs: `0.130338`
- Corr: `0.9911796808169983`

### Test Metrics
- MAE: `0.016963`
- RMSE: `0.024815`
- MaxAbs: `0.125105`
- MAE / mean(|tau|): `0.1600480444898778`
- Corr: `0.9913988549429613`

## Joint5

### Basis Terms
- `1`
- `sin(q5)`
- `cos(q5)`
- `sin(q2+q3+q4+q5)`
- `cos(q2+q3+q4+q5)`

### Train Metrics
- MAE: `0.316425`
- RMSE: `0.380108`
- MaxAbs: `1.382415`
- Corr: `0.6767579476759676`

### Test Metrics
- MAE: `0.317005`
- RMSE: `0.380726`
- MaxAbs: `1.362417`
- MAE / mean(|tau|): `0.3898280378631478`
- Corr: `0.6774328870464758`

## Joint6

### Basis Terms
- `1`
- `sin(q6)`
- `cos(q6)`

### Train Metrics
- MAE: `0.815025`
- RMSE: `1.024764`
- MaxAbs: `2.966508`
- Corr: `0.1086349727315749`

### Test Metrics
- MAE: `0.813698`
- RMSE: `1.025607`
- MaxAbs: `2.975430`
- MAE / mean(|tau|): `0.9972036647518028`
- Corr: `0.1112168235218301`
