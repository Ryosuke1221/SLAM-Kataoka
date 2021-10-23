
- 1
- 2
- 3

4. 444
5. 555
6. 666

- [x] Task1
- [] Task2
- [ ] Task3

text

>reference

| 左寄せ | 中央寄せ | 右寄せ |
|:------|:--------:|-------:|
| 左    | 中央     | 右　   |

|  TH  |  TH  |
| ---- | ---- |
|  TD  |  TD  |
|  TD  |  TD  |

Click [reference_page1](https://help.github.com/ja/github/writing-on-github/basic-writing-and-formatting-syntax)

Click [reference_page2](https://cpp-learning.com/readme/)

Click [reference_page3](https://cpp-learning.com/wp-content/uploads/2019/06/README_Template.html)

# プログラム操作手順
# 0. プログラム概要
- 主に3つのソリューションを順番に用いる。<br>
- 出力結果はそれぞれ、テキスト(もしくはフォルダ)に書き出される。<br>
- 出力結果のファイルを、適切なフォルダに手動でコピペすることで、次のソリューションに渡す。

## 0.1 言葉の定義
- ターゲット点群：位置合わせにおいて動かない方の点群。
- ソース点群：ターゲット点群に対して位置合わせされる点群。

# 1. SLAM-kataoka
- 属性付き点群から、各フレーム間での大域的位置合わせ結果を出力する。<br>
- 属性付き点群、各フレーム間での大域的位置合わせ結果から、各フレーム間での局所的位置合わせ結果を出力する。

## 1.1 ディレクトリ構造
```
SLAM-kataoka/
  ├source/
  ├build/
  ├data/
```

## 1.2 大域的位置合わせ

### 1.2.1 設定パラメータ
```
data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\parameter_vecvec.csv
```
↑このファイルに、大域的位置合わせを行う際に用いるパラメータをまとめている。<br>
1つのパラメータに複数の値を指定すると、それぞれの場合の結果を出力する。<br>
例：パラメータ1とパラメータ2を2つずつ指定すると、合計で4(=2*2)つの位置合わせ結果が出力される。<br>
<br>

```
data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\pattern_vecvec.csv
```
↑このファイルにて、結果1つ分の計算に用いるパラメータの組み合わせを行ごとにまとめている。<br>
例：パラメータ1とパラメータ2を2つずつ指定すると、合計で4(=2*2)つの位置合わせ結果の出力となり、このファイルは4行の構成となる。<br>
※一番上の行にてパラメータ名を指定している。

### 1.2.2 出力形式
```
data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\
```
↑このフォルダ直下に"時刻_手法の種類"というフォルダが作成され、その中に出力結果が保存される。<br>
例：2021年2月2日6時8分55秒736ミリ秒に従来手法を用いると、結果は20210202_0608_55_736_conventionalに出力される。
<br>
#### 1.2.2.1 位置合わせ結果：変位情報
位置合わせの変位は"時刻_手法_output.csv"というファイル名で出力される。<br>

出力結果の.csvは、excelで閲覧することで表形式で確認できる。<br>
以下に設定パラメータの説明を示す。<br>
| パラメータ | 意味 |
| :--: | :--: |
|  voxel_size | TD |
|  radius_normal_FPFH | TD |
| radius_FPFH | TD |
| MaxCorrespondenceDistance_SAC | TD |
| SimilarityThreshold_SAC | TD |
| InlierFraction_SAC | TD |
| MaximumIterations_SAC | TD |
| NumberOfSamples_SAC | TD |
| CorrespondenceRandomness_SAC | TD |
| th_nearest_nir | TD |
| th_rank_rate_nir | TD |
| th_nearest_velodyne | TD |
| th_rank_rate_velodyne | TD |
| th_nearest_fpfh | TD |
| num_nearest_fpfh | TD |
| th_rank_rate_fpfh | TD |
| i_method_rigidTransformation | TD |
| th_geometricConstraint | TD |

 <br>


以下に出力の説明を示す。<br>
| 出力 | 意味 |
| :--: | :--: |
| i_tgt | TD |
| i_src | TD |
| isProposed | TD |
| b_usedNIR | TD |
| b_usedVelodyne | TD |
| b_usedFPFH | TD |
| X | TD |
| Y | TD |
| Z | TD |
| Roll | TD |
| Pitch | TD |
| Yaw | TD |
| isConverged | TD |
| corr_nir_size | TD |
| corr_velodyne_size | TD |
| corr_fpfh_size | TD |
| corr_output_size | TD |
| e_euqulid | TD |
| e_error_PointCloudDistance | TD |
| median_nearest | TD |
| e_error_beta | TD |
| e_error_angle_normal | TD |

#### 1.2.2.2 位置合わせ結果：点群データ
出力された位置合わせ結果を2点群間に適用した後の点群を.pcd形式で出力する。
位置合わせを行った点群の全組み合わせが出力される。<br>
例：0フレームの点群をターゲット点群、1フレームの点群をソース点群、手法を従来手法とした際の点群は
```
tgt00src01_result_00conventional.pcd
```
となる。

### 1.2.3 実行手順