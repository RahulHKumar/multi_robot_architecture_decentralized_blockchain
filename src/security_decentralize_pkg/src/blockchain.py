#!/usr/bin/env python

import hashlib

class GeekCoinBlock:

    def __init__(self, previous_block_hash, transaction_list):

        self.previous_block_hash = previous_block_hash
        self.transaction_list = transaction_list

        self.block_data = "-".join(transaction_list)+"-"+previous_block_hash
        self.block_hash = hashlib.sha256(self.block_data.encode()).hexdigest()

class Blockchain:
    def __init__(self):
        self.chain = []
        self.generate_genesis_block()

    def generate_genesis_block(self):
        self.chain.append(GeekCoinBlock("0", ['Genesis Block']))

    def create_block_from_transaction(self, transaction_list):
        previous_block_hash = self.last_block.block_hash
        self.chain.append(GeekCoinBlock(previous_block_hash, transaction_list))

    def display_chain(self):
        for i in range(len(self.chain)):
            # print("Data {}: {}".format(i+1,self.chain[i].block_data))
            # print("Hash {}: {}".format(i+1,self.chain[i].block_hash))
            return str(self.chain[i+1].block_hash)

    @property
    def last_block(self):
        return self.chain[-1]
