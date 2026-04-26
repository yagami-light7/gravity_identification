# 三角基函数重力拟合报告

- 输入 CSV: `E:\RoboMaster\mec_arm\gravity_identification\data\mc02_capture_cleaned.csv`
- 样本数量: `8692`
- 测试集抽样步长: `5`
- Ridge 正则系数: `0.01`

## Joint1

### Basis Terms

- `1`
- `sin(q1)`
- `cos(q1)`

### Train Metrics

- MAE: `0.283720`
- RMSE: `0.338768`
- MaxAbs: `1.040584`
- Corr: `0.31210569229778706`

### Test Metrics

- MAE: `0.283330`
- RMSE: `0.338529`
- MaxAbs: `1.012029`
- MAE / mean(|tau|): `0.7617506406129013`
- Corr: `0.30663896149870723`

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

- MAE: `0.623470`
- RMSE: `0.825580`
- MaxAbs: `3.720882`
- Corr: `0.9912925244812737`

### Test Metrics

- MAE: `0.622510`
- RMSE: `0.824332`
- MaxAbs: `3.429673`
- MAE / mean(|tau|): `0.11822872452546547`
- Corr: `0.9913223238290636`

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

- MAE: `0.589119`
- RMSE: `0.708416`
- MaxAbs: `2.173749`
- Corr: `0.8001841572236578`

### Test Metrics

- MAE: `0.584537`
- RMSE: `0.702664`
- MaxAbs: `2.006916`
- MAE / mean(|tau|): `0.07896476265883655`
- Corr: `0.8020635746156796`

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

- MAE: `0.016799`
- RMSE: `0.022232`
- MaxAbs: `0.098767`
- Corr: `0.8590955438651218`

### Test Metrics

- MAE: `0.016831`
- RMSE: `0.022314`
- MaxAbs: `0.101817`
- MAE / mean(|tau|): `0.33527679989831005`
- Corr: `0.85912253745137`

## Joint5

### Basis Terms

- `1`
- `sin(q5)`
- `cos(q5)`
- `sin(q2+q3+q4+q5)`
- `cos(q2+q3+q4+q5)`

### Train Metrics

- MAE: `0.281386`
- RMSE: `0.349732`
- MaxAbs: `1.183239`
- Corr: `0.26986993627823314`

### Test Metrics

- MAE: `0.280286`
- RMSE: `0.349744`
- MaxAbs: `1.165494`
- MAE / mean(|tau|): `0.30030341389539406`
- Corr: `0.26049723187385215`

## Joint6

### Basis Terms

- `1`
- `sin(q6)`
- `cos(q6)`

### Train Metrics

- MAE: `0.861338`
- RMSE: `1.064673`
- MaxAbs: `3.265894`
- Corr: `0.2913668554422458`

### Test Metrics

- MAE: `0.859038`
- RMSE: `1.061055`
- MaxAbs: `3.109241`
- MAE / mean(|tau|): `0.9636107375057725`
- Corr: `0.2896049249354515`
