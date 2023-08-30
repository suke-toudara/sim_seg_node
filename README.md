# sim_seg_node
疑似セグメンテーションデータ作成用トピック

## param
sim_seg_node/include/sim_seg_node/に設定ファイルがある、パラメータは以下の通り
```
class_num : クラス数(int)
mode : 疑似トピックの作成方法　(k_means or random_seg_img)
pub_seg_topic : 受け取る画像トピック
sub_img_topic : 出力するsegmentationトピック
```
