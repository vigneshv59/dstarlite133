import heapq
import graphics

class PriorityQueue:
    def __init__(self):
        self._heap = []
        self._present_elems = {}
        self._size = 0

    def enqueue(self, elem):
        heapq.heappush(self._heap, elem)

        self._present_elems[elem[1]] = True
        self._size += 1

    def dequeue(self):
        while len(self._heap) > 0:
            elem = heapq.heappop(self._heap)

            if self._present_elems[elem[1]]:
                self._size -= 1
                self._present_elems[elem[1]] = False
                return elem

            self._present_elems[elem[1]] = False


    def remove(self, elem):
        if self._present_elems.get(elem, False):
            self._present_elems[elem] = False
            self._size -= 1

    def peek(self):
        while len(self._heap) > 0:
            elem = self._heap[0]

            if self._present_elems[elem[1]]:
                return elem

            heapq.heappop(self._heap)

    def __len__(self):
        return self._size

    def __contains__(self, item):
        return self.present_elems.get(elem[1], False)
