#!/usr/bin/env python3
import pykakasi

class RomanConverter:
    def __init__(self):
        self.kks = pykakasi.kakasi()
        self.kks.setMode('J', 'a')
        self.kks.setMode('H', 'a')
        self.kks.setMode('K', 'a')
        self.convertor = kks.getConverter()
        
    def convert(self, text):
        return self.converter.do(text)
        
