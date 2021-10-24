
<!-- 4. 444
5. 555
6. 666

- [x] Task1
- [ ] Task3

text

>reference

| 左寄せ | 中央寄せ | 右寄せ |
|:------|:--------:|-------:|
| 左    | 中央     | 右　   |


Click [reference_page1](https://help.github.com/ja/github/writing-on-github/basic-writing-and-formatting-syntax)

Click [reference_page2](https://cpp-learning.com/readme/)

Click [reference_page3](https://cpp-learning.com/wp-content/uploads/2019/06/README_Template.html) -->

# プログラム操作手順
# 0. プログラム概要

- 主に3つのソリューションを順番に用いる。<br>

- 出力結果はそれぞれ、テキスト(もしくはフォルダ)に書き出される。<br>

- 出力結果のファイルを、適切なフォルダに手動でコピペすることで、次のソリューションに渡す。


## 0.1 言葉の定義

- ターゲット点群：位置合わせにおいて動かない方の点群。

- ソース点群：ターゲット点群に対して位置合わせされる点群。

- NIR：near infra-red、近赤外線

# 1. SLAM-kataoka


## 1.1 ディレクトリ構造
```
SLAM-kataoka/
  ├source/
  ├build/
  ├data/
```

## 1.2 点群の前処理


### 1.2.1 点群データの場所

点群データは、
```
data\data_test_03GlobalFeatureRegistration\03_all
```
に配置する。<br>

なお、
```
data\data_test_03GlobalFeatureRegistration\00_nir
```
には近赤外情報を計測したフレームのみの点群、<br>

```
data\data_test_03GlobalFeatureRegistration\01_velodyne
```
には近赤外情報を計測したフレーム以外の点群を保存している。<br>


### 1.2.2 点群データの生成

センサから得た点群データ(.csv)を、当プログラムを適用できる形(.pcd)に変換する。<br>

#### 1.2.2.1 .csvから.pcd(途中1)を生成

入力データの.csvは
```
data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
/05_NarahaWinter202001\_01Generation
```
に配置する。<br>

まず、Velodyneのみを計測に用いたフレームの点群の、velodyneデータを生成する。<br>

- build/Project.slnをVisual Studioで開く。

- test_PointcloudGenerationをスタートアッププロジェクトに指定する。

- デバッグなしで実行。<br>
コマンドプロンプトが立ち上がる。

- NarahaWinter202001に該当する番号を入力してエンターを押す。<br>
(本資料作成時では8。)

- 0: PCDGenerationを選ぶ。<br>

- 0: VELO_PCAPを選ぶ。<br>
XXX (Frame 0000).csvが読み込まれ、.pcdが生成される。<br>
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_01Generation時刻_Output_XYZI_VELO_PCAP
に保存される。<br>

続いて、VelodyneとNIRセンサを計測に用いたフレームの点群の、velodyneデータを生成する。

- NarahaWinter202001に該当する番号を入力まで共通。

- 0: PCDGenerationを選ぶ。<br>

- 1: VELO_withNIRを選ぶ。<br>
XXXvelo.csvが読み込まれ、.pcdが生成される。<br>
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_01Generation時刻_Output_XYZI_VELO_withNIR
に保存される。<br>

続いて、VelodyneとNIRセンサを計測に用いたフレームの点群の、NIRデータを生成する。

- NarahaWinter202001に該当する番号を入力まで共通。

- 0: PCDGenerationを選ぶ。<br>

- 2: NIRを選ぶ。<br>
XXXnir.csvが読み込まれ、.pcdが生成される。
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_01Generation時刻_Output_XYZI_NIR
に保存される。<br>


#### 1.2.2.2 .pcd(途中1)から.pcd(途中2)を生成

入力データの.pcd(1.2.2.1で生成したもの)は
```
data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
/05_NarahaWinter202001\_02Filtering
```
に配置する。<br>

まず、velodyneデータから、位置合わせに不要な箇所の点群を除去する。

- NarahaWinter202001に該当する番号を入力まで共通。

- 1: Filteringを選ぶ。<br>

- 0: FilteringVelodyneを選ぶ。<br>
XXX (Frame 0000).pcdが読み込まれ、.pcd(途中2)が生成される。<br>
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_02Filtering時刻_Output
に保存される。<br>

続いて、NIRデータから、位置合わせに不要な箇所の点群を除去する。

- NarahaWinter202001に該当する番号を入力まで共通。

- 1: Filteringを選ぶ。<br>

- 1: FilteringNIRを選ぶ。<br>
XXXvelo.pcdとXXXvnir.pcdが読み込まれ、.pcd(途中2)が生成される。<br>
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_02Filtering時刻_Output
に保存される。<br>


#### 1.2.2.3 .pcd(途中2)から.pcd(完成)を生成

入力データの.pcd(1.2.2.2で生成したもの)は
```
data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
/05_NarahaWinter202001\_03Combination
```
に配置する。<br>

まず、Velodyneのみを計測に用いたフレームの点群から、手法に適用できる形式の点群データを生成する。<br>

- NarahaWinter202001に該当する番号を入力まで共通。

- 2: Combination of Velodyne and NIRを選ぶ。<br>

- 0: Combine Velodyne onlyを選ぶ。<br>
XXX (Frame 0000).pcdとXXXvelo.pcdが読み込まれ、.pcd(完成)が生成される。<br>
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_03Combination_Output
に保存される。<br>

- select delete them or not  1:yes  0:no<br>
既に出力フォルダ内に別の.pcdが存在する場合にこの選択肢が出る。<br>
yesを選択することでこれらを削除することができる。

- Do you delete it realy?  yes:1  no:0<br>
古い.pcdを削除して問題ないなら、yesを選択する。

続いて、VelodyneとNIRセンサを計測に用いたフレームの点群から、手法に適用できる形式の点群データを生成する。<br>

- NarahaWinter202001に該当する番号を入力まで共通。

- 2: Combination of Velodyne and NIRを選ぶ。<br>

- 1: Combine Velodyne and NIRを選ぶ。<br>
XXX (Frame 0000).pcdとXXXvelo.pcdが読み込まれ、.pcd(完成)が生成される。
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_03Combination_Output
に保存される。<br>

- select delete them or not  1:yes  0:no<br>
既に出力フォルダ内に別の.pcdが存在する場合にこの選択肢が出る。<br>
既にVelodyneのみを計測に用いたフレームのの点群を生成しているはずなので、noを選択する。


## 1.3 大域的位置合わせ
- 属性付き点群、各フレーム間での大域的位置合わせ結果から、各フレーム間での局所的位置合わせ結果を出力する。


### 1.3.1 設定パラメータ
```
data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\parameter_vecvec.csv
```
↑このファイルに、大域的位置合わせを行う際に用いるパラメータをまとめている。<br>

1つのパラメータに複数の値を指定すると、それぞれの場合の結果を出力する。<br>

例：パラメータ1とパラメータ2を2つずつ指定すると、合計で4(=2*2)つの位置合わせ結果が出力される。<br>

<br>

以下に設定パラメータの説明を示す。<br>
| パラメータ | 意味 |
| :-- | :-- |
|  voxel_size | VGFのボクセル1つあたりの辺の大きさ。このボクセルを単位として点群を低解像度化する |
|  radius_normal_FPFH | 法線推定において、ある注目点の法線を求める際に周囲の点群を考慮する範囲。 |
| radius_FPFH | 近傍点を探索する範囲。 |
| MaxCorrespondenceDistance_SAC | 収束判定の際、インライアを決める距離の閾値として用いられる。 |
| SimilarityThreshold_SAC | bad pair rejectionにて使用。i番目のpairに関して、i-1番目のquery点間の距離、match点間の距離を出して、両者の比(1より小さくなるよう分子分母を入替)がSimilarityThresholdより小さくなった場合にpairを削除する。 |
| InlierFraction_SAC | 収束判定の際、(インライアの点の数/ソース点群のサイズ)の値がこの値より大きければ、収束と判定される。 |
| MaximumIterations_SAC | この回数を超えても収束と判定されなければ、剛体変換の算出は失敗となる。 |
| NumberOfSamples_SAC | ソース点群の中からこの数だけランダムに点を選び、点対応を求める際の候補とする。 |
| CorrespondenceRandomness_SAC | FPFH空間での近傍点探索にて、この数だけ近傍点をピックアップする。近傍点の中からランダムに最近傍点を決め、最終的な点対応としている． |
| th_nearest_nir | 近赤外情報を踏まえた近傍点計算時の、近赤外値の閾値。 |
| th_rank_rate_nir | TD |
| th_nearest_velodyne | TD |
| th_rank_rate_velodyne | TD |
| th_nearest_fpfh | TD |
| num_nearest_fpfh | TD |
| th_rank_rate_fpfh | TD |
| i_method_rigidTransformation | TD |
| th_geometricConstraint | TD |

<!-- パラメータが未記入の箇所がある。xx -->
 <br>

```
data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\pattern_vecvec.csv
```
↑このファイルにて、結果1つ分の計算に用いるパラメータの組み合わせを行ごとにまとめている。<br>

例：パラメータ1とパラメータ2を2つずつ指定すると、合計で4(=2*2)つの位置合わせ結果の出力となり、このファイルは4行の構成となる。<br>

※一番上の行にてパラメータ名を指定している。


### 1.3.2 出力形式
```
data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\
```
↑このフォルダ直下に"時刻_手法の種類"というフォルダが作成され、その中に出力結果が保存される。<br>

例：2021年2月2日6時8分55秒736ミリ秒に従来手法を用いると、結果は20210202_0608_55_736_conventionalに出力される。
<br>


#### 1.3.2.1 位置合わせ結果：変位情報

位置合わせの変位は"時刻_手法_output.csv"というファイル名で出力される。<br>

出力結果の.csvは、excelで閲覧することで表形式で確認できる。<br>

以下に出力の説明を示す。<br>
| 出力 | 意味 |
| :-- | :-- |
| i_tgt | ターゲット点群のフレーム。 |
| i_src | ソース点群のフレーム。 |
| isProposed | 従来手法なら0、提案手法なら1。 |
| b_usedNIR | 近赤外情報を用いたかどうか。 |
| b_usedVelodyne | Velodyne反射強度情報を用いたかどうか 。 |
| b_usedFPFH | FPFH情報を用いたかどうか。 |
| X | X軸方向の変位。 |
| Y | Y軸方向の変位。 |
| Z | Z軸方向の変位。 |
| Roll | X軸方向の回転。 |
| Pitch | X軸方向の回転。 |
| Yaw | X軸方向の回転。 |
| isConverged | 位置合わせが収束したかどうか。<br>収束は1、失敗は0。 |
| corr_nir_size | 近赤外情報の点対応算出時の点対応数。 |
| corr_velodyne_size | Velodyne反射強度情報の点対応算出時の点対応数。 |
| corr_fpfh_size | FPFH情報の点対応算出時の点対応数。 |
| corr_output_size | 位置合わせ計算に用いた点対応数。 |
| e_euqulid | 並進方向変位の真値とのエラー。 |
| e_error_PointCloudDistance | 位置合わせ後の点の座標とのエラーの平均値。 |
| median_nearest | 位置合わせ後のターゲット点群とソース点群の近傍点距離の中央値。 |
| e_error_beta | 回転方向の真値とのエラー(ロボットの方向ベクトルのねじれ方向の角度誤差)。 |
| e_error_angle_normal | 回転方向の真値とのエラー(ロボットの方向ベクトルの角度誤差)。 |
| time_elapsed | 処理全体にかかった時間。 |

#### 1.3.2.2 位置合わせ結果：点群データ

出力された位置合わせ結果を2点群間に適用した後の点群を.pcd形式で出力する。<br>

位置合わせを行った点群の全組み合わせが出力される。<br>

この点群を目視することで、位置合わせの成否を判断できる。<br>
<!-- xx赤と緑、どっちがtgtでどっちがsrcかを説明すべき。
 -->

<!-- 画像で例を示したい。
<img src = "https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/918362/b657bc10-7dc0-b89f-6a10-c2114c2cce96.png" alt="出力された点群" title="出力された点群" width="400" height="100" /> -->

例：0フレームの点群をターゲット点群、1フレームの点群をソース点群、手法を従来手法とした際の点群は
```
tgt00src01_result_00conventional.pcd
```
となる。


### 1.3.3 実行準備

- data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_03Combination_Output
にある点群をdata\data_test_03GlobalFeatureRegistration\03_all
に配置する。


### 1.3.4 実行手順

- build/Project.slnをVisual Studioで開く。

- test_03GlobalFeatureFegistraionをスタートアッププロジェクトに指定する。

- デバッグなしで実行。<br>
コマンドプロンプトが立ち上がる。

- VariParamaters_GlobalRegistrationに該当する番号を入力してエンターを押す。<br>
(本資料作成時では14。)

- do you create new pattern?  Yes:1  No:0<br>
計算したいパラメータの組み合わせを更新した場合のみYes(1)、更新が無ければNo(0)を選択する。<br>
※data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\parameter_vecvec.csvの値を参照してdata\data_test_03GlobalFeatureRegistration\Result_01varyParameters\pattern_vecvec.csvが更新される。

- press 1 and Enter if you have closed file<br>
1を押してエンターを押すと計算が開始する。
<!-- 処理中の画面に関しても説明したい。xx -->


## 1.4 局所的位置合わせ

- 属性付き点群、各フレーム間での大域的位置合わせ結果から、各フレーム間での局所的位置合わせ結果を出力する。<br>
この手法はICPをベースにしている。

### 1.4.1 設定パラメータ

```
data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\_Input\parameter_vecvec.csv
```
↑このファイルに、局所的位置合わせを行う際に用いるパラメータをまとめている。<br>

1つのパラメータに複数の値を指定すると、それぞれの場合の結果を出力する。<br>

以下に設定パラメータの説明を示す。<br>

| パラメータ | 意味 |
| :-- | :-- |
| MaximumIterations | ICPの計算を行う回数の最大値。 |
| MaxCorrespondenceDistance | 近傍点算出の際の距離の閾値。 |
| EuclideanFitnessEpsilon | 収束判定時の、近傍点距離平均の変化量に関する閾値。 |
| TransformationEpsilon | 収束判定時の、変位の変化量に関する閾値。 |
| penalty_chara | A |
| weight_dist_chara | A |
| th_successOfGlobalRegistration_distance | A |
<!-- パラメータが未記入の箇所がある。xx -->
 <br>


```
data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\_Input\pattern_vecvec.csv
```
↑このファイルにて、結果1つ分の計算に用いるパラメータの組み合わせを行ごとにまとめている。<br>


### 1.4.2 出力形式

```
data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\
```
↑このフォルダ直下に"時刻_手法の種類_ICP"というフォルダが作成され、その中に出力結果が保存される。<br>

例：2021年2月26日23時11分18秒835ミリ秒に従来手法を用いると、結果は20210226_2311_18_835_conventional_ICPに出力される。
<br>


#### 1.4.2.1 位置合わせ結果：変位情報

位置合わせの変位は"時刻_手法_output.csv"というファイル名で出力される。<br>

出力結果の.csvは、excelで閲覧することで表形式で確認できる。<br>

設定パラメータの一番下に、どの大域的位置合わせ結果を用いたかを"時刻_手法"の形で示す。<br>

以下に出力の説明を示す。<br>

| 出力 | 意味 |
| :-- | :-- |
| i_tgt | ターゲット点群のフレーム。 |
| i_src | ソース点群のフレーム。 |
| isProposed | 従来手法なら0、提案手法なら1。 |
| X | X軸方向の変位。 |
| Y | Y軸方向の変位。 |
| Z | Z軸方向の変位。 |
| Roll | X軸方向の回転。 |
| Pitch | X軸方向の回転。 |
| Yaw | X軸方向の回転。 |
| isConverged | 位置合わせが収束したかどうか。<br>収束は1、失敗は0。 |
| e_euqulid | 並進方向変位の真値とのエラー。 |
| e_error_PointCloudDistance | 位置合わせ後の点の座標とのエラーの平均値。 |
| e_error_PointCloudDistance_median | 位置合わせ後の点の座標とのエラーの中央値。 |
| e_error_beta | 回転方向の真値とのエラー(ロボットの方向ベクトルのねじれ方向の角度誤差)。 |
| e_error_angle_normal | 回転方向の真値とのエラー(ロボットの方向ベクトルの角度誤差)。 |
| mean_nearest | 位置合わせ後のターゲット点群とソース点群の近傍点距離の平均値。 |
| median_nearest | 位置合わせ後のターゲット点群とソース点群の近傍点距離の中央値。 |
| b_estimatedSuccess | 位置合わせの成否の推定値。<br>成功は1、失敗は0。 |
| time_elapsed | 処理全体にかかった時間。 |


#### 1.4.2.2 位置合わせ結果：点群データ

出力された位置合わせ結果を2点群間に適用した後の点群を.pcd形式で出力する。<br>

位置合わせを行った点群の全組み合わせが出力される。<br>

この点群を目視することで、位置合わせの成否を判断できる。<br>

<!-- xx赤と緑、どっちがtgtでどっちがsrcかを説明すべき。
 -->

<!-- 画像で例を示したい。
<img src = "https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/918362/b657bc10-7dc0-b89f-6a10-c2114c2cce96.png" alt="出力された点群" title="出力された点群" width="400" height="100" /> -->

例：0フレームの点群をターゲット点群、1フレームの点群をソース点群、手法を従来手法とした際の点群は
```
tgt00src01_result_00conventional_ICP.pcd
```
となる。


### 1.4.3 実行準備

```
data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\_input\
```
に、大域位置合わせの実行結果のフォルダを配置する。<br>

フォルダ名の例：時刻_conventional


### 1.4.4 実行手順

- build/Project.slnをVisual Studioで開く。

- test_03GlobalFeatureFegistraionをスタートアッププロジェクトに指定する。

- デバッグなしで実行。<br>
コマンドプロンプトが立ち上がる。

- ICP_VariParamatersに該当する番号を入力してエンターを押す。<br>
(本資料作成時では16。)

- do you create new pattern?  Yes:1  No:0<br>
計算したいパラメータの組み合わせを更新した場合のみYes(1)、更新が無ければNo(0)を選択する。<br>
※data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\\_Input\parameter_vecvec.csvの値を参照してdata\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\\_Input\pattern_vecvec.csvが更新される。

- press 1 and Enter if you have closed file<br>
1を押してエンターを押すと計算が開始する。
