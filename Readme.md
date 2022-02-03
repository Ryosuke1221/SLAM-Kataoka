# プログラム操作手順
# 0. プログラム概要

- 主に3つのソリューションを順番に用いる。<br>

- 出力結果はそれぞれ、テキスト(もしくはフォルダ)に書き出される。<br>

- 出力結果のファイルを、適切なフォルダに手動でコピペすることで、次のソリューションに渡す。
<br><br>

## 0.1 言葉の定義

- ターゲット点群：位置合わせにおいて動かない方の点群。

- ソース点群：ターゲット点群に対して位置合わせされる点群。

- NIR：near infra-red、近赤外線
<br><br>

## 0.2 関連リンク

- 真の軌跡を計算する方法： [手動位置合わせ](https://github.com/Ryosuke1221/SLAM-Kataoka/tree/master/source/test_HandRegistration)
<br><br>

# 1. SLAM-kataoka


## 1.1 ディレクトリ構造
```
SLAM-kataoka/
  ├source/
  ├build/
     ├Project.sln
  ├data/
     ├data_test_03GlobalFeatureRegistration/
        ├03_all/
     ├data_test_PointcloudGeneration/
        ├03_PCDGeneration_fromCSV/
           ├05_NarahaWinter202001/
              ├_01Generation/
              ├_02Filtering/
              ├_03Combination/
              ├_03Combination_Output/
```
<br>

## 1.2 点群の前処理


### 1.2.1 点群データの場所

点群データは、
```
data\data_test_03GlobalFeatureRegistration\03_all
```
に配置する。<br><br>

なお、
```
data\data_test_03GlobalFeatureRegistration\00_nir
```
には近赤外情報を計測したフレームのみの点群、<br>

```
data\data_test_03GlobalFeatureRegistration\01_velodyne
```
には近赤外情報を計測したフレーム以外の点群を保存している。<br><br>


### 1.2.2 点群データの生成

センサから得た点群データ(.csv)を、当プログラムを適用できる形(.pcd)に変換する。<br><br>

#### 1.2.2.1 .csvから.pcd(途中1)を生成

入力データの.csvは
```
data\data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\_01Generation
```
に配置する。<br><br>

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
に保存される。<br><br>

続いて、VelodyneとNIRセンサを計測に用いたフレームの点群の、velodyneデータを生成する。

- NarahaWinter202001に該当する番号を入力まで共通。

- 0: PCDGenerationを選ぶ。<br>

- 1: VELO_withNIRを選ぶ。<br>
XXXvelo.csvが読み込まれ、.pcdが生成される。<br>
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_01Generation時刻_Output_XYZI_VELO_withNIR
に保存される。<br><br>

続いて、VelodyneとNIRセンサを計測に用いたフレームの点群の、NIRデータを生成する。

- NarahaWinter202001に該当する番号を入力まで共通。

- 0: PCDGenerationを選ぶ。<br>

- 2: NIRを選ぶ。<br>
XXXnir.csvが読み込まれ、.pcdが生成される。
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV\05_NarahaWinter202001\\_01Generation時刻_Output_XYZI_NIR
に保存される。<br><br>


#### 1.2.2.2 .pcd(途中1)から.pcd(途中2)を生成

入力データの.pcd(1.2.2.1で生成したもの)は
```
data_test_PointcloudGeneration\03_PCDGeneration_fromCSV\05_NarahaWinter202001\_02Filtering
```
に配置する。<br><br>

まず、velodyneデータから、位置合わせに不要な箇所の点群を除去する。

- NarahaWinter202001に該当する番号を入力まで共通。

- 1: Filteringを選ぶ。<br>

- 0: FilteringVelodyneを選ぶ。<br>
XXX (Frame 0000).pcdが読み込まれ、.pcd(途中2)が生成される。<br>
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_02Filtering時刻_Output
に保存される。<br><br>

続いて、NIRデータから、位置合わせに不要な箇所の点群を除去する。

- NarahaWinter202001に該当する番号を入力まで共通。

- 1: Filteringを選ぶ。<br>

- 1: FilteringNIRを選ぶ。<br>
XXXvelo.pcdとXXXvnir.pcdが読み込まれ、.pcd(途中2)が生成される。<br>
生成された点群は、data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_02Filtering時刻_Output
に保存される。<br><br>


#### 1.2.2.3 .pcd(途中2)から.pcd(完成)を生成

入力データの.pcd(1.2.2.2で生成したもの)は
```
data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
/05_NarahaWinter202001\_03Combination
```
に配置する。<br><br>

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
古い.pcdを削除して問題ないなら、yesを選択する。<br><br>

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
既にVelodyneのみを計測に用いたフレームのの点群を生成しているはずなので、noを選択する。<br><br>


## 1.3 大域的位置合わせ
- 属性付き点群、各フレーム間での大域的位置合わせ結果から、各フレーム間での局所的位置合わせ結果を出力する。<br><br>


### 1.3.1 設定パラメータ
```
data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\parameter_vecvec.csv
```
↑このファイルに、大域的位置合わせを行う際に用いるパラメータをまとめている。<br>

1つのパラメータに複数の値を指定すると、それぞれの場合の結果を出力する。<br>

複数の値を設定したい場合は、所定のセルの右のセルに値を順次追加していく。<br>

例：パラメータ1とパラメータ2を2つずつ指定すると、合計で4(=2*2)つの位置合わせ結果が出力される。<br><br>

以下に設定パラメータの説明を示す。<br>
| パラメータ | 意味 |
| :-- | :-- |
| voxel_size | VGFのボクセル1つあたりの辺の大きさ。このボクセルを単位として点群を低解像度化する |
| radius_normal_FPFH | 法線推定において、ある注目点の法線を求める際に周囲の点群を考慮する範囲。 |
| radius_FPFH | 近傍点を探索する範囲。 |
| MaxCorrespondenceDistance_SAC | 収束判定の際、インライアを決める距離の閾値として用いられる。 |
| SimilarityThreshold_SAC | bad pair rejectionにて使用。i番目のpairに関して、i-1番目のquery点間の距離、match点間の距離を出して、両者の比(1より小さくなるよう分子分母を入替)がSimilarityThresholdより小さくなった場合にpairを削除する。 |
| InlierFraction_SAC | 収束判定の際、(インライアの点の数/ソース点群のサイズ)の値がこの値より大きければ、収束と判定される。 |
| MaximumIterations_SAC | この回数を超えても収束と判定されなければ、剛体変換の算出は失敗となる。 |
| NumberOfSamples_SAC | ソース点群の中からこの数だけランダムに点を選び、点対応を求める際の候補とする。 |
| CorrespondenceRandomness_SAC | FPFH空間での近傍点探索にて、この数だけ近傍点をピックアップする。近傍点の中からランダムに最近傍点を決め、最終的な点対応としている． |
| th_nearest_nir | 近赤外情報を踏まえた近傍点計算における、近傍とみなすかを与える閾値。 |
| th_rank_rate_nir | 空間的ばらつきを考慮した点対応集合の選別(近赤外情報)において、ばらつきのパラメータが小さい点対応を選別する際、小さい方から数えてどれほどの割合<br>(0 < th_rank_rate_nir < 1)<br>を選別するかを与えるパラメータ。 |
| th_nearest_velodyne | LiDAR反射強度情報を踏まえた近傍点計算における、近傍とみなすかを与える閾値。 |
| th_rank_rate_velodyne | 空間的ばらつきを考慮した点対応集合の選別(LiDAR反射強度情報)において、ばらつきのパラメータが小さい点対応を選別する際、小さい方から数えてどれほどの割合<br>(0 < th_rank_rate_velodyne < 1)<br>を選別するかを与えるパラメータ。 |
| th_nearest_fpfh | FPFH情報を踏まえた近傍点計算における、近傍とみなすかを与える閾値。 |
| num_nearest_fpfh | FPFH情報を踏まえた近傍点計算における、近傍点数の最大値を与える閾値。 |
| th_rank_rate_fpfh | 空間的ばらつきを考慮した点対応集合の選別(FPFH情報)において、ばらつきのパラメータが小さい点対応を選別する際、小さい方から数えてどれほどの割合<br>(0 < th_rank_rate_fpfh < 1)<br>を選別するかを与えるパラメータ。 |
| i_method_rigidTransformation | 幾何学的拘束を考慮した点対応集合の選別において、出力候補の点対応集合同士を比較する際の評価値の計算方法を、数パターンある中から指定するためのパラメータ。<br>基本的に固定値2(点対応集合の)を用いている。
 |
| th_geometricConstraint | 幾何学的拘束を考慮した点対応集合の選別において、用いている閾値。<br>この閾値の値が大きくなるほど基準が厳しくなり、出力される点対応集合のサイズが小さくなる。 |
 <br>

```
data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\pattern_vecvec.csv
```
↑このファイルにて、結果1つ分の計算に用いるパラメータの組み合わせを行ごとにまとめている。<br>

例：パラメータ1とパラメータ2を2つずつ指定すると、合計で4(=2*2)つの位置合わせ結果の出力となり、このファイルは4行の構成となる。<br>

※一番上の行にてパラメータ名を指定している。<br><br>


### 1.3.2 出力形式
```
data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\
```
↑このフォルダ直下に"時刻_手法の種類"というフォルダが作成され、その中に出力結果が保存される。<br>

例：2021年2月2日6時8分55秒736ミリ秒に従来手法を用いると、結果は20210202_0608_55_736_conventionalに出力される。<br><br>


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
<br>

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
となる。<br><br>


### 1.3.3 位置合わせするフレームのスキップ
- 位置合わせが明らかに失敗してしまうフレームが分かっている場合は、事前の入力によって、そのフレームとの位置合わせをスキップすることができる。
- スキップしたいフレームは、以下のファイルに入力することによって指定する。
```
data\data_test_03GlobalFeatureRegistration\ignore_frame_list.csv
```
- このファイルを表計算ソフトで開き、例えばフレームが0~4まである場合は、

| 0 |   |
| - | - |
| 1 |   |
| 2 |   |
| 3 |   |
| 4 |   |

となるように記入する。<br>
- スキップしたいフレームのセル位置に'-1'(マイナス1)を記入することで、位置合わせをスキップすることができる。

- 例えば、以下のように入力すると、1フレームと4フレームでの位置合わせをスキップすることができる。

| 0 |   |
| - | - |
| 1 | -1|
| 2 |   |
| 3 |   |
| 4 | -1|

※フレームの組み合わせのスキップを行わない場合も、このファイルを準備する必要がある。
<br><br>

### 1.3.4 位置合わせするフレームの組み合わせのスキップ
- 位置合わせが明らかに失敗してしまうフレームの組み合わせが分かっている場合は、事前の入力によって、そのフレーム間の位置合わせをスキップすることができる。
- スキップしたいフレームの組み合わせは、以下のファイルに入力することによって指定する。
```
data\data_test_03GlobalFeatureRegistration\ignore_framePair_matrix.csv
```
- このファイルを表計算ソフトで開き、例えばフレームが0~4まである場合は、

| -   | 0   | 1   | 2   | 3   |  4  | 
| --- | --- | --- | --- | --- | --- | 
| 0   |  -  |     |     |     |     | 
| 1   |  -  |  -  |     |     |     | 
| 2   |  -  |  -  |  -  |     |     | 
| 3   |  -  |  -  |  -  |  -  |     | 
| 4   |  -  |  -  |  -  |  -  |  -  | 

となるように記入する。<br>
- スキップしたいフレームの組み合わせのセル位置に'-1'(マイナス1)を記入することで、位置合わせをスキップすることができる。

- 例えば、以下のように入力すると、0-1と0-4での組み合わせの位置合わせをスキップすることができる。

| -   | 0   | 1   | 2   | 3   |  4  | 
| --- | --- | --- | --- | --- | --- | 
| 0   |  -  | -1  |     |     | -1  | 
| 1   |  -  |  -  |     |     |     | 
| 2   |  -  |  -  |  -  |     |     | 
| 3   |  -  |  -  |  -  |  -  |     | 
| 4   |  -  |  -  |  -  |  -  |  -  | 

<br>
※フレームの組み合わせのスキップを行わない場合も、このファイルを準備する必要がある。



<br><br>

### 1.3.5 位置合わせ 実行準備

- data_test_PointcloudGeneration\03_PCDGeneration_fromCSV<br>
\05_NarahaWinter202001\\_03Combination_Output
にある点群をdata\data_test_03GlobalFeatureRegistration\03_all
に配置する。<br><br>


### 1.3.6 位置合わせ 実行手順

- build\Project.slnをVisual Studioで開く。

- test_03GlobalFeatureFegistraionをスタートアッププロジェクトに指定する。

- デバッグなしで実行。<br>
コマンドプロンプトが立ち上がる。

- VariParamaters_GlobalRegistrationに該当する番号を入力してエンターを押す。<br>
(本資料作成時では14。)

- do you create new pattern?  Yes:1  No:0<br>
計算したいパラメータの組み合わせを更新した場合のみYes(1)、更新が無ければNo(0)を選択する。<br>
※data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\parameter_vecvec.csvの値を参照してdata\data_test_03GlobalFeatureRegistration\Result_01varyParameters\pattern_vecvec.csvが更新される。<br>

- Do you calculate new pointcloud ? (calcUniquePointOfFPFHInSomeRadius):  yes:1  no:0<br>
点群データを新しくしたり、FPFH特徴量を計算するためのパラメータ(voxel_size,radius_normal_FPFH,radius_FPFH)を更新した場合は1を選択する。<br>
1を選択した場合は、FPFH特徴量の計算の過程の値を始めから計算し、またその値をファイルに出力する。<br>
0を選択した場合は、FPFH特徴量の計算の過程の値をファイルから読み込んで、全体の計算にかかる時間を節約する。
   
- press 1 and Enter if you have closed file<br>
1を押してエンターを押すと計算が開始する。<br><br>
<!-- 処理中の画面に関しても説明したい。xx -->


### 1.3.7 位置合わせ結果比較 実行準備

- data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\_Comparison\
に大域位置合わせの実行結果のフォルダを配置する。<br>フォルダ名の例：時刻_conventional
<br><br>


### 1.3.8 位置合わせ結果比較 実行手順

- build\Project.slnをVisual Studioで開く。

- test_03GlobalFeatureFegistraionをスタートアッププロジェクトに指定する。

- デバッグなしで実行。<br>
コマンドプロンプトが立ち上がる。

- CompareGlobalRegistrationに該当する番号を入力してエンターを押す。<br>
(本資料作成時では15。)

- 位置合わせ結果の比較の結果が
data\data_test_03GlobalFeatureRegistration\Result_01varyParameters\\_Comparison\に出力される。<br>

- 以下に出力の説明を示す。<br>

| 出力 | 意味 |
| :-- | :-- |
| b_isProposed | 提案手法を用いたかどうか(FPFHを用いていれば0、大域的位置合わせを用いていれば1)。<br>→なぜかどの場合でも1になってしまうバグが存在する。 |
| num_allFramePairs | 位置合わせの計算を行ったフレームの組の総数。 |
| num_succeededFramePairs | 位置合わせが成功したと思われるフレームの組の総数。 |
| succeededFramePairs | 位置合わせが成功したと思われるフレームの組。 |
| biggestCluster | succeededFramePairsを繋ぎ合わせて、相対的な変位が計算できるフレームの組み合わせの中で、一番大きい物。<br>→num_succeededFramePairsが0の時に値がおかしくなるバグが存在する。 |
| size_biggestCluster | biggestClusterのサイズ。 |
| second_biggestCluster | succeededFramePairsを繋ぎ合わせて、相対的な変位が計算できるフレームの組み合わせの中で、二番目に大きい物。 |
| frames_notContainded | biggestClusterに含まれていないフレーム。<br>→たまに計算されないバグが存在する。 |
<br>


## 1.4 局所的位置合わせ

- 属性付き点群、各フレーム間での大域的位置合わせ結果から、各フレーム間での局所的位置合わせ結果を出力する。<br>
この手法はICPをベースにしている。
<br><br>

### 1.4.1 設定パラメータ

```
data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\_Input\parameter_vecvec.csv
```
↑このファイルに、局所的位置合わせを行う際に用いるパラメータをまとめている。<br>

1つのパラメータに複数の値を指定すると、それぞれの場合の結果を出力する。<br>

複数の値を設定したい場合は、所定のセルの右のセルに値を順次追加していく。<br>

以下に設定パラメータの説明を示す。<br>

| パラメータ | 意味 |
| :-- | :-- |
| MaximumIterations | ICPの計算を行う回数の最大値。 |
| MaxCorrespondenceDistance | 近傍点算出の際の距離の閾値。 |
| EuclideanFitnessEpsilon | 収束判定時の、近傍点距離平均の変化量に関する閾値。 |
| TransformationEpsilon | 収束判定時の、変位の変化量に関する閾値。 |
| penalty_chara | 位置合わせ後のエラー値の算出時にて、近傍点の属性クラスがそれぞれ異なる場合(例えば熱源属性クラスと水溜まり属性クラスなど)、<br>"点1つあたりのエラー値 = (点間距離 +  penalty_chara) / 点対応数の合計"<br>とする。 |
| weight_dist_chara | 位置合わせ処理の近傍点探索時にて、近傍点探索の基準となるソース点群の点が希少クラスである場合、<br>"探索距離の最大値 = 元々の最大値 * weight_dist_chara"<br>とする。これにより希少クラスの点を注目点とした際に、探索距離を拡張する。 |
| th_successOfGlobalRegistration_distance | 大域的位置合わせの結果においてe_error_PointCloudDistanceの値がこれよりも小さい場合のみにICPを行うとものしている。 |
 <br>


```
data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\_Input\pattern_vecvec.csv
```
↑このファイルにて、結果1つ分の計算に用いるパラメータの組み合わせを行ごとにまとめている。<br><br>


### 1.4.2 出力形式

```
data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\
```
↑このフォルダ直下に"時刻_手法の種類_ICP"というフォルダが作成され、その中に出力結果が保存される。<br>

例：2021年2月26日23時11分18秒835ミリ秒に従来手法を用いると、結果は20210226_2311_18_835_conventional_ICPに出力される。<br><br>


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
<br>

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
となる。<br><br>


### 1.4.3 位置合わせ 実行準備

```
data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\_input\
```
に、大域位置合わせの実行結果のフォルダを配置する。<br>

フォルダ名の例：時刻_conventional<br><br>


### 1.4.4 位置合わせ 実行手順

- build\Project.slnをVisual Studioで開く。

- test_03GlobalFeatureFegistraionをスタートアッププロジェクトに指定する。

- デバッグなしで実行。<br>
コマンドプロンプトが立ち上がる。

- ICP_VariParamatersに該当する番号を入力してエンターを押す。<br>
(本資料作成時では16。)

- do you create new pattern?  Yes:1  No:0<br>
計算したいパラメータの組み合わせを更新した場合のみYes(1)、更新が無ければNo(0)を選択する。<br>
※data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\\_Input\parameter_vecvec.csvの値を参照してdata\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\\_Input\pattern_vecvec.csvが更新される。

- press 1 and Enter if you have closed file<br>
1を押してエンターを押すと計算が開始する。<br><br>


### 1.4.5 位置合わせ結果比較 実行準備

- data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\\_Comparison\
に局所的位置合わせの実行結果のフォルダを配置する。<br>フォルダ名の例：時刻_conventional_ICP
<br><br>


### 1.4.6 位置合わせ結果比較 実行手順

- build\Project.slnをVisual Studioで開く。

- test_03GlobalFeatureFegistraionをスタートアッププロジェクトに指定する。

- デバッグなしで実行。<br>
コマンドプロンプトが立ち上がる。

- CompareICPに該当する番号を入力してエンターを押す。<br>
(本資料作成時では17。)

- 位置合わせ結果の比較の結果が
data\data_test_03GlobalFeatureRegistration\Result_02_ICP_varyParameters\\_Comparison\に出力される。<br>

- 以下に出力の説明を示す。<br>

| 出力 | 意味 |
| :-- | :-- |
| filename_GlobalRegistration: | この局所的位置合わせの計算に用いた、大域的位置合わせのファイル名。 |
| b_isProposed | 提案手法を用いたかどうか(FPFHとICPを用いていれば0、提案手法の大域的位置合わせと局所的位置合わせを用いていれば1)。 |
| th_successOfICP_distance: | 局所的位置合わせの結果においてe_error_PointCloudDistanceの値がこれよりも小さい場合は成功とみなす。<br>基本的に用いているのは定数3で、プログラム中で指定されている。  |
| num_allFramePairs | 位置合わせの計算を行ったフレームの組の総数。 |
| num_succeededFramePairs | 位置合わせが成功したと思われるフレームの組の総数。 |
| succeededFramePairs | 位置合わせが成功したと思われるフレームの組。 |
| biggestCluster | succeededFramePairsを繋ぎ合わせて、相対的な変位が計算できるフレームの組み合わせの中で、一番大きい物。 |
| size_biggestCluster | biggestClusterのサイズ。 |
| second_biggestCluster | succeededFramePairsを繋ぎ合わせて、相対的な変位が計算できるフレームの組み合わせの中で、二番目に大きい物。 |
| frames_notContainded | biggestClusterに含まれていないフレーム。<br>→たまに計算されないバグが存在する。 |
<br>


# 2. ポーズ調整

- 属性付き点群、各フレーム間での局所的的位置合わせ結果から、ロボット(センサ)の走行した軌跡を出力する。<br>
ここでは、従来から広く用いられてきたポーズ調整を用いる。

## 2.1 ディレクトリ構造
```
Open3D_loop_closure-master/
  ├source/
  ├build/
     ├Open3D.sln
  ├_InputOutput/
     ├__202102/
        ├_comparison/
        ├_input/
        ├_output/
        ├_pointcloud/
        ├parameter_vecvec.csv
        ├pattern_vecvec.csv
        ├transformation_fin.csv   
```
<br>

## 2.2 設定パラメータ

```
Open3D_loop_closure-master\_InputOutput\__202102\parameter_vecvec.csv
```
↑このファイルに、ポーズ調整を行う際に用いるパラメータをまとめている。<br>

1つのパラメータに複数の値を指定すると、それぞれの場合の結果を出力する。<br>

複数の値を設定したい場合は、所定のセルの右のセルに値を順次追加していく。<br>

以下に設定パラメータの説明を示す。<br>

| パラメータ | 意味 |
| :-- | :-- |
| edge_inf_threshold | ポーズ調整の情報行列算出に関係する閾値で、この値が大きいほど各変位の位置合わせ精度の優劣が付きにくくなる。 |
| max_correspondence_distance | ロボット位置のノードをエッジで繋ぐか否かを判断する時に使われるらしい。 |
| edge_prune_threshold | ロボット位置のノードをエッジで繋ぐか否かを判断する時に使われるらしい。<br>この値が大きいほど、エッジとして見做されなくなる。 |
 <br>


```
Open3D_loop_closure-master\_InputOutput\__202102\pattern_vecvec.csv
```
↑このファイルにて、結果1つ分の計算に用いるパラメータの組み合わせを行ごとにまとめている。<br><br>


## 2.3 出力形式

```
Open3D_loop_closure-master\_InputOutput\__202102\_output
```
↑このフォルダ直下に出力結果が保存される。<br>

ポーズ調整後の変位は"時刻_手法の種類_optimization_output.csv"というファイル名で出力される。<br>

出力結果の.csvは、excelで閲覧することで表形式で確認できる。<br><br>


### 2.3.1 ポーズ調整：ポーズ調整前の軌跡

Trajectory(before):の下に、ポーズ調整前の軌跡が出力される。<br>

1行あたりの軌跡データは、フレーム、X、Y、Z、Roll、Pitch、Yawで表現される。<br>

※この軌跡は、局所的位置合わせ結果の変位の、若いフレーム間の変位を優先的に選んで計算される。<br><br>


### 2.3.2 ポーズ調整：変位情報

以下に出力の説明を示す。<br>
※スキップされたフレームの変位には-1が代入される。

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
| IsSkiped | フレームが計算において考慮されたかどうか(1ならスキップ)。 |
| e_euqulid_relative | 前のフレームからの変位の真値とのユークリッドエラー。 |
| e_euqulid_absolute | 真のロボット位置とのユークリッド距離。 |
| e_error_beta_relative | 前のフレームからの変位における回転方向の真値とのエラー(ロボットの方向ベクトルのねじれ方向の角度誤差)。 |
| e_error_angle_normal_relative | 真のロボット位置における回転方向の真値とのエラー(ロボットの方向ベクトルのねじれ方向の角度誤差)。 |
| e_error_beta_absolute | 前のフレームからの変位における回転方向の真値とのエラー(ロボットの方向ベクトルの角度誤差)。 |
| e_error_angle_normal_absolute | 真のロボット位置における回転方向の真値とのエラー(ロボットの方向ベクトルの角度誤差)。 |
| e_error_PointCloudDistance | ポーズ調整後の点の座標とのエラーの平均値。 |
| e_error_PointCloudDistance_median | ポーズ調整後の点の座標とのエラーの中央値。 |
<br>


### 2.3.3 ポーズ調整：ポーズ調整後の軌跡

Trajectory:の下に、ポーズ調整後の軌跡が出力される。<br>

1行あたりの軌跡データは、フレーム、X、Y、Z、Roll、Pitch、Yawで表現される。<br><br>


### 2.3.4 ポーズ調整：フレーム全体を通した評価値

ポーズ調整後の軌跡(Trajectory:の下)の下に、フレーム全体を通した軌跡の評価値が出力される。<br>

| 出力 | 意味 |
| :-- | :-- |
| e_euqulid_relative_mean: | e_euqulid_relativeの平均値(フレームごとの点群サイズによる重みづけ無し)。 |
| e_euqulid_absolute_mean: | e_euqulid_absoluteの平均値(フレームごとの点群サイズによる重みづけ無し)。 |
| e_error_PointCloudDistance_map: | e_error_PointCloudDistanceの平均値(フレームごとの点群サイズによる重みづけ有り)。 |
<br>


## 2.4 位置合わせ 実行準備

- Open3D_loop_closure-master以下をいずれかのフォルダにコピーする。

- Open3D_loop_closure-master\buildの中身を全て削除する。

- CMake(cmake-gui)を起動する。

- CMakeでの処理<br>
キャッシュを削除する。<br>
scrとbuildのフォルダを設定する。<br>
configureを選ぶ。<br>
searchにpyと入れる。<br>
python関係の選択肢が出てくるので、全部チェックを外す。※項目は3つくらいだった。<br>
configureを選ぶ。configureが通るはず。エラーのポップアップが出なければ成功。
generateを選ぶ。<br>
Open Projectを選ぶ。これでVisual Studioが起動する。

- Visual Studioでの、バッチビルドまでの処理<br>
Debugx64を選択する。<br>
ビルドからバッチビルドを選択する。<br>
Releaseとdebugとついているものを全部選択する。<br>
ビルドを選択する。<br>
暫く待って、結果が46正常終了、40失敗となれば恐らく成功である。この数が違うと後々エラーが出る可能性がある。

```
考えられるエラー原因：
・最初のcmakeのキャッシュ削除を忘れる．
・別のソースコードを入れたり，既存のものを書き換えたりすること．
```

- Visual Studioでの、バッチビルドから先の処理<br>
ソリューションエクスプローラから、ソリューションOpen3Dに右クリックをする。<br>
プロパティを選択する。<br>
共通プロパティ、スタートアッププロジェクト、シングルスタートアップ、TestPoseGraphをそれぞれ選択する。<br>
デバッグ無しで実行を選択する(エラーは多分出ない。出ても以下の処理を実行すると上手くいく場合がある。)。<br>
Releasex64に変更する。
ソリューションエクスプローラからTestPoseGraphのプルダウンを開く。
Source Filesに右クリック、追加、既存の項目を選択し、
Open3D_loop_closure-master\src\Testにある
Optimization_FeatureRegistration.cpp,Optimization_FeatureRegistration.h,
Optimization_FPFH.cpp,Optimization_FPFH.h,
TimeString.cpp,TimeString.h
をそれぞれ追加する。
デバッグ無しで実行する。

- 計算に用いる点群をOpen3D_loop_closure-master<br>\\_InputOutput\\__202102\\_pointcloud
に配置する。

- 真の軌跡データを
Open3D_loop_closure-master\\_InputOutput\\__202102
に配置する。<br><br>


## 2.5 位置合わせ 実行手順

- build\Open3D.slnをVisual Studioで開く。

- test\TestPoseGraphをスタートアッププロジェクトに指定する。

- デバッグなしで実行。<br>
コマンドプロンプトが立ち上がる。

- select method:  0:calculation_varyParameters  1:compare_Opitmization<br>
0を選択する。

- do you create new pattern?  Yes:1  No:0<br>
計算したいパラメータの組み合わせを更新した場合のみYes(1)、更新が無ければNo(0)を選択する。<br>
※Open3D_loop_closure-master\\_InputOutput\\__202102\parameter_vecvec.csvの値を参照してOpen3D_loop_closure-master\\_InputOutput\\__202102\pattern_vecvec.csvが更新される。<br>

- press 1 and Enter if you have closed file<br>
1を押してエンターを押すと計算が開始する。<br><br>
<!-- 処理中の画面に関しても説明したい。xx -->


## 2.6 位置合わせ結果比較 実行準備
- Open3D_loop_closure-master\\_InputOutput\\__202102\\_comparisonに、__202102\_output\に出力された出力ファイルを配置する。<br><br>


## 2.7 位置合わせ結果比較 実行手順
- build\Open3D.slnをVisual Studioで開く。

- test\TestPoseGraphをスタートアッププロジェクトに指定する。

- デバッグなしで実行。<br>
コマンドプロンプトが立ち上がる。

- select method:  0:calculation_varyParameters  1:compare_Opitmization<br>
1を選択する。

- ポーズ調整結果の比較の結果が
Open3D_loop_closure-master\\_InputOutput\\__202102\\_comparison\に"opt日付_comparison.csv"という名前で出力される。<br>

- 以下に出力の説明を示す。<br>

| 出力 | 意味 |
| :-- | :-- |
| Removed_FramePairs | ポーズ調整の過程で計算から除外されたフレーム。 |
| b_isProposed | 提案手法を用いたかどうか(FPFHとICPを用いていれば0、提案手法の大域的位置合わせと局所的位置合わせを用いていれば1)。 |
| e_error_PointCloudDistance"i" | iフレームの点群におけるe_error_PointCloudDistanceの値。 |
| e_euqulid_relative_mean | e_euqulid_relative_meanの値。 |
| e_euqulid_absolute_mean | e_euqulid_absolute_meanの値。 |
| e_error_PointCloudDistance_map | e_error_PointCloudDistance_mapの値。 |
<br>
