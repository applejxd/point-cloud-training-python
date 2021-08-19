from collections import namedtuple
from operator import itemgetter
from pprint import pformat


# namedtuple はクラスを返す関数：引数はそれぞれクラス名・属性
class Node(namedtuple("Node", "location left_child right_child")):
    """

    """
    def __repr__(self):
        """
        print の書式を定義
        """
        return pformat(tuple(self))


class KDTree:
    """
    平衡 k-d 木
    """
    def __init__(self, point_list):
        # 次元
        self._k = len(point_list[0])
        self.tree = self._make_tree(point_list)

    def _make_tree(self, point_list, depth: int = 0):
        """
        再帰的に k-d tree を作成
        cf. https://en.wikipedia.org/wiki/K-d_tree

        :param point_list:  作成対象の点群
        :param depth:       現在のツリーの深さ（初期値 0, 最大 log_2(点群数)）
        :return:            k-d 木
        """
        if not point_list:
            return None

        # 点の次元：すべての点が同次元と仮定

        # 深さに応じすべての軸を巡回するように軸選択(mod 計算)
        axis = depth % self._k

        # axis 方向にソート
        point_list.sort(key=itemgetter(axis))
        # 中点を tree の分割点として選択
        median = len(point_list) // 2

        # ノード作成
        return Node(
            # 中点
            location=point_list[median],
            # 子ツリー（小さい側）
            left_child=self._make_tree(point_list[:median], depth + 1),
            # 子ツリー（大きい側）
            right_child=self._make_tree(point_list[median + 1:], depth + 1),
        )

    def get_leaf(self, point):
        return self._get_child(point, self.tree, 0, [])

    def _get_child(self, point, node, depth, ans):
        if node is None:
            node = Node(location=point, left_child=None, right_child=None)
            return ans

        axis = depth % self._k
        if point[axis] <= node.location[axis]:
            ans.append("L")
            return self._get_child(point, node.left_child, depth + 1, ans)
        elif point[axis] > node.location[axis]:
            ans.append("R")
            return self._get_child(point, node.right_child, depth + 1, ans)
        else:
            raise ValueError("Invalid input point!")

    def add_point(self, point):
        branch = self.tree
        for side in path:
            if side is "L":
                branch = branch.left_child
            elif side is "R":
                branch = branch.right_child
            else:
                raise ValueError("Invalid input path!")

        branch = Node(location=point, left_child=None, right_child=None)

    def search_nearest(self, point):
        pass


def main():
    """Example usage"""
    point_list = [(7, 2), (5, 4), (9, 6), (4, 7), (8, 1), (2, 3)]
    kd_tree = KDTree(point_list)
    leaf = kd_tree.get_leaf((2, 7))
    print(leaf)


if __name__ == "__main__":
    main()
