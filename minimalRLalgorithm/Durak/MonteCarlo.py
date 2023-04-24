import random
import numpy as np
import math
import json
from cards import Deck, Card
from Durakmain import DurakEnv


# removed x variables since the node having a stored state works the same.

class TupleError(Exception):
    '''Raise when tuple is not present in conditional probability dictionary.
    This is here mainly for students to catch their own errors.
    '''

    def __init__(self, name):
        super().__init__("Error: Tuple not present in conditional probability dictionary of Node " + name)


class Node:
    '''A node in the bayesian network.
    Please read all comments in this class.
    '''
    nodes = {}  # Static object: use this to access any existing node

    # REMEMBER TO RESET THIS if you would like to use a different JSON file
    def __init__(self, name):
        self.name = name  # number
        self.strategy = "blank"  # string
        self.player = "blank"  # string
        self.parent = 0  # parent names
        self.children = []  # list of children names
        self.wins = 0  # U(n) number of wins during past simulation on this node
        self.simulations = 1  # N(n)  number of simulation that was applied on this node

        Node.nodes[name] = self


def topological():
    '''Return list of Node names in topological order.
    The code here performs a topological sort on the nodes.
    '''
    visited = set()
    top_sorted = []

    # Helper function to DFS over all children
    def visit(node_name):
        visited.add(node_name)
        for child_name in Node.nodes[node_name].children:
            if child_name not in visited:
                visit(child_name)
        # At this point, we have visited all children
        top_sorted.append(node_name)

    for node_name in Node.nodes:
        if node_name not in visited:
            visit(node_name)
    return top_sorted[::-1]


def parse_file(filename):
    '''Parse JSON file of bayesian network.
    JSON key-value pairs:
    "Name"         -- Name of the node.
    "Parents"      -- List of names of parent nodes.
                      Conditionals are given in order of this list.
    "Children"     -- List of names of child nodes.
    '''
    with open(filename, 'r') as f:
        data = json.load(f)

        for node_data in data:
            node = Node(node_data['Name'])
            node.strategy = node_data['Strategy']
            node.player = node_data['Player']
            node.parent = node_data['Parents']
            node.children = node_data['Children']
            node.wins = node_data['Wins']
            node.simulations = node_data['Simulations']


def UCB1_Score(n):
    '''n is the node name'''
    C = 2
    UCB1 = (n.wins / n.simulations) + (C * math.sqrt(math.log(Node.nodes[n.parent].simulations) / n.simulations))
    return UCB1


def Selection(children, mult):
    scores = {}
    for i in range(len(children)):
        if not mult and Node.nodes[children[i]].strategy == "Multiple":
            continue
        if DurakEnv.FIRST_ATTACK and Node.nodes[children[i]].strategy == "Pass":
            continue
        state = Node.nodes[children[i]]
        score = UCB1_Score(state)
        if state not in scores:
            scores[state] = score
    max_score = 0
    max_state = []
    for state, score in scores.items():
        if score >= max_score:
            if score == max_score:
                max_state.append(state)
                max_score = score
            else:
                max_score = score
                max_state = [state]
    if len(max_state) == 1:
        return max_state[0]
    strategy = random.choice(max_state)
    return strategy


def Multiple(hand):
    max_length = 0
    rank = 0
    for i in range(0, len(hand)):
        s = hand[i]
        get_indexes = lambda x, xs: [i for (y, i) in zip(xs, range(len(xs))) if x == y]
        options = get_indexes(s, hand)
        if len(options) > max_length:
            max_length = len(options)
            rank = s
    return rank


# possibly add input number of cards?
def generate_hand(hand, trump, testhand_len):
    deck = Deck()
    deck.shuffle()
    new_hand = []
    while len(new_hand) < testhand_len:
        card = deck.deal()
        if card not in hand:
            new_hand.append(card)
    DurakEnv.reorder([new_hand], trump)
    return new_hand


def parse_tree(start, multhand, multtest, tree):
    strategies = []
    mult = True
    if start.children:
        if start.player == "Attacker" and tree == "Attacker":
            mult = multtest
        elif start.player == "Defender" and tree == "Attacker":
            mult =multhand
        elif start.player == "Attacker" and tree == "Defender":
            mult = multhand
        else:
            mult = multtest
        strategy = Selection(start.children, mult)
        strategies.append(strategy)
        out = parse_tree(strategy,multhand, multtest, tree)
        if out:
            for s in out:
                strategies.append(s)
    return strategies


def expandnode(parentnode):
    final = max(topological())
    hnode = Node(final + 1)
    mnode = Node(final + 2)
    pnode = Node(final + 3)
    lnode = Node(final + 4)
    parentnode.children.append(hnode.name)
    parentnode.children.append(lnode.name)
    parentnode.children.append(mnode.name)
    parentnode.children.append(pnode.name)
    hnode.parent = parentnode.name
    lnode.parent = parentnode.name
    pnode.parent = parentnode.name
    mnode.parent = parentnode.name
    p = parentnode.player
    if p == "Attacker":
        hnode.player = "Defender"
        mnode.player = "Defender"
        pnode.player = "Defender"
        lnode.player = "Defender"
    else:
        hnode.player = "Attacker"
        mnode.player = "Defender"
        pnode.player = "Defender"
        lnode.player = "Attacker"
    hnode.strategy = "Highest"
    lnode.strategy = "Lowest"
    mnode.strategy = "Multiple"
    pnode.strategy = "Pass"


def remove_leaf(start, nodes):
    children = start.children
    if start.name in Node.nodes[start.parent].children and start.name != nodes[-1].name:
        Node.nodes[start.parent].children.pop(Node.nodes[start.parent].children.index(start.name))
    start.children = []
    for child in children:
        if not Node.nodes[child].children:
            del Node.nodes[child]
        else:
            remove_leaf(Node.nodes[child], nodes)


def back_propogate(a_wins, nodes, Leaf, deck, card_hand_values=None):
    for n in nodes:
        n.simulations += 1
    if a_wins:
        for n in nodes:
            if n.player == "Attacker":
                n.wins += 1
            if n.strategy == "Highest" and deck > 26:
                n.wins -= 0.5
            if n.strategy == "Pass":
                if n.player == "Defender":
                    if deck > 26 and min(card_hand_values) > 10:
                        n.wins += 0.25
                    elif deck > 13 and min(card_hand_values) > 10:
                        n.wins += 0.1
    if not a_wins:
        for n in nodes:
            if n.player == "Defender":
                n.wins += 1
            if n.strategy == "Highest" and deck > 26:
                n.wins -= 0.5
            if n.strategy == "Pass":
                if n.player == "Attacker":
                    if deck > 26 and min(card_hand_values) > 10:
                        n.wins += 0.75
                    elif deck > 13 and min(card_hand_values) > 10:
                        n.wins += 0.5
                    elif deck > 0 and min(card_hand_values) > 10:
                        n.wins -= 0.25
    if not Leaf:
        if nodes[-1].name not in [1,2,3,4]:
            remove_leaf(nodes[-1], nodes)
    if Leaf:
        children = nodes[-1].children
        for child in children:
            Node.nodes[child].simulations = 1
            Node.nodes[child].wins = 1

def choose_multiple(selects, trump, hand):
    probs = {}
    playable = selects[0]
    for card in playable:
        print(card, hand)
        if hand[card].suit() == trump.suit():
            probs[card] = 1 / 6
        else:
            probs[card] = 1 / len(playable)
    tot = 0
    for k, v in probs.items():
        tot += v
    for k, v in probs.items():
        probs[k] = v/tot
    num = random.random()
    curr = 0
    for k,v in probs.items():
        if num > curr and num < (curr + v):
            return k
        curr = curr + v
def select_highest(hand, testhand, defending, trump, cards_played):
    if not cards_played:
        ranked_hand = DurakEnv.get_hand_ranks([hand], trump, cards_played)
    else:
        if (defending and cards_played[-1][1] == "") or (not defending and cards_played[-1][1] != ""):
            ranked_hand = DurakEnv.get_hand_ranks([hand], trump, cards_played)
        elif (defending and cards_played[-1][1] != "") or (not defending and cards_played[-1][1] == ""):
            ranked_hand = DurakEnv.get_hand_ranks([testhand], trump, cards_played)
    return ranked_hand[1]

def select_lowest(hand, testhand, defending, trump, cards_played):
    if not cards_played:
        ranked_hand = DurakEnv.get_hand_ranks([hand], trump, cards_played)
    else:
        if (defending and cards_played[-1][1] == "") or (not defending and cards_played[-1][1] != ""):
            ranked_hand = DurakEnv.get_hand_ranks([hand], trump, cards_played)
        elif (defending and cards_played[-1][1] != "") or (not defending and cards_played[-1][1] == ""):
            ranked_hand = DurakEnv.get_hand_ranks([testhand], trump, cards_played)
    return ranked_hand[0]

def Simulate(hand, testhand, defending, trump, cards_played, Strategy, deck):
    curr = False
    if curr == defending:
        h = hand
    else:
        h = testhand
    pc = DurakEnv.count_playable(h)
    nodes = []
    high = 0
    reset = False
    low = 0
    multiple = 0
    pass_c = 0
    pass_var = False
    if defending:
        defender_hand = hand
        attacker_hand = testhand
    else:
        defender_hand = testhand
        attacker_hand = hand
    upper_limit = DurakEnv.find_upper_limit(defender_hand, attacker_hand, True)
    if Strategy[0].name == 1:
        high += 1
    elif Strategy[0].name == 2:
        low += 1
    elif Strategy[0].name == 3:
        multiple += 1
    else:
        pass_c +=1
    for i in range(len(Strategy)):
        chosen_strategy = Strategy[i]
        if reset:
            h = hand
            if cards_played and ((defending and cards_played[-1][1] != "") or (not defending and cards_played[-1][1] == "")):
                h = testhand
            mult = DurakEnv.mult_possible(DurakEnv.get_amounts_per_number(h))
            if prev.children:
                chosen_strategy = Selection(prev.children, mult)
                prev = chosen_strategy
            else:
                break
        nodes.append(chosen_strategy)
        if chosen_strategy.strategy == "Highest":
            select = select_highest(hand, testhand, defending, trump, cards_played)

            # select largest ranked card
        elif chosen_strategy.strategy == "Lowest":
            select = select_lowest(hand, testhand, defending, trump, cards_played)
        elif chosen_strategy.strategy == "Multiple":
            select = 0
            if not cards_played:
                ranked_hand = DurakEnv.get_hand_ranks([hand], trump, cards_played, mult = True)
                selects = ranked_hand[2]
                if not selects:
                    reset = True
                    if chosen_strategy.children:
                        chosen_strategy = Selection(chosen_strategy.children, False)
                        prev = chosen_strategy
                        if chosen_strategy.strategy == "Highest":
                            select = select_highest(hand, testhand, defending, trump, cards_played)
                        elif chosen_strategy.strategy == "Lowest":
                            select = select_lowest(hand, testhand, defending, trump, cards_played)
                        else:
                            pass_var = True
                    else:
                        break
                else:
                    pos_sel = []
                    ranks = []
                    print(ranked_hand)
                    for k in selects:
                        pos_sel.append(choose_multiple(selects[k], trump, hand))
                        ranks.append(hand[pos_sel[-1]].rank())
                        if hand[pos_sel[-1]].suit() == trump.suit():
                            ranks[-1] += 13
                    if len(pos_sel) > 1:
                        if high > low:
                            select = pos_sel[ranks.index(max(ranks))]
                        elif low > high:
                            select = pos_sel[ranks.index(min(ranks))]
                        else:
                            if len(pos_sel) == 1:
                                select = pos_sel[0]
                            else:
                                select = random.choice(pos_sel)
                    else:
                        select = pos_sel[0]
            else:
                if (defending and cards_played[-1][1] == "") or (not defending and cards_played[-1][1] != ""):
                    ranked_hand = DurakEnv.get_hand_ranks([hand], trump, cards_played, mult = True)
                    selects = ranked_hand[2]
                    if not selects:
                        reset = True
                        if chosen_strategy.children:
                            chosen_strategy = Selection(chosen_strategy.children, False)
                            prev = chosen_strategy
                            if chosen_strategy.strategy == "Highest":
                                select = select_highest(hand, testhand, defending, trump, cards_played)
                            elif chosen_strategy.strategy == "Lowest":
                                select = select_lowest(hand, testhand, defending, trump, cards_played)
                            else:
                                pass_var = True
                        else:
                            break
                    else:
                        pos_sel = []
                        ranks = []
                        print(ranked_hand)
                        for k in selects:
                            pos_sel.append(choose_multiple(selects[k], trump, hand))
                            ranks.append(hand[pos_sel[-1]].rank())
                            if hand[pos_sel[-1]].suit() == trump.suit():
                                ranks[-1] += 13
                        if len(pos_sel) > 1:
                            if high > low:
                                select = pos_sel[ranks.index(max(ranks))]
                            elif low > high:
                                select = pos_sel[ranks.index(min(ranks))]
                            else:
                                if len(pos_sel) == 1:
                                    select = pos_sel[0]
                                else:
                                    select = random.choice(pos_sel)
                        else:
                            select = pos_sel[0]
                elif (defending and cards_played[-1][1] != "") or (not defending and cards_played[-1][1] == ""):
                    ranked_hand = DurakEnv.get_hand_ranks([testhand], trump, cards_played, mult = True)
                    selects = ranked_hand[2]
                    if not selects:
                        reset = True
                        if chosen_strategy.children:
                            chosen_strategy = Selection(chosen_strategy.children, False)
                            prev = chosen_strategy
                            if chosen_strategy.strategy == "Highest":
                                select = select_highest(hand, testhand, defending, trump, cards_played)
                            elif chosen_strategy.strategy == "Lowest":
                                select = select_lowest(hand, testhand, defending, trump, cards_played)
                            else:
                                pass_var = True
                        else:
                            break
                    else:
                        pos_sel = []
                        ranks = []
                        print(ranked_hand)
                        for k in selects:
                            pos_sel.append(choose_multiple(selects[k], trump, testhand))
                            ranks.append(testhand[pos_sel[-1]].rank())
                            if testhand[pos_sel[-1]].suit() == trump.suit():
                                ranks[-1] += 13
                        if len(pos_sel) > 1:
                            if high > low:
                                select = pos_sel[ranks.index(max(ranks))]
                            elif low > high:
                                select = pos_sel[ranks.index(min(ranks))]
                            else:
                                if len(pos_sel) == 1:
                                    select = pos_sel[0]
                                else:
                                    select = random.choice(pos_sel)
                        else:
                            select = pos_sel[0]
        else:
            pass_var = True
        if pass_var:
            ranked_hand = DurakEnv.get_only_hand_ranks([hand], trump, cards_played, passing=True)
            if defending:
                back_propogate(True, nodes, True, deck, ranked_hand)
            else:
                back_propogate(False, nodes, True, deck, ranked_hand)
        else:
            if (not cards_played and not defending) or (defending and cards_played[-1][1] == "") or (not defending and cards_played[-1][1] != ""):
                card = hand.pop(select)
                DurakEnv.reset_playable(hand)
                if cards_played:
                    if cards_played[-1][1] == "":
                        cards_played[-1] = (cards_played[-1][0], card)
                    else:
                        cards_played.append((card, ""))
                else:
                    cards_played.append((card, ""))
                DurakEnv.find_playable(cards_played, testhand, not defending, trump)
                pc = DurakEnv.count_playable(testhand)
            else:
                card = testhand.pop(select)
                DurakEnv.reset_playable(testhand)
                if cards_played:
                    if cards_played[-1][1] == "":
                        cards_played[-1] = (cards_played[-1][0], card)
                    else:
                        cards_played.append((card, ""))
                else:
                    cards_played.append((card, ""))
                DurakEnv.find_playable(cards_played, hand, defending, trump)
                pc = DurakEnv.count_playable(hand)
            if pc == 0:
                if defending:
                    back_propogate(True, nodes, False, deck)
                    # attacker wins
                else:
                    back_propogate(False, nodes, False, deck)
                    # defender wins
                break
            if len(cards_played) == upper_limit and cards_played[-1][1] != "":
                # defender wins
                back_propogate(False, nodes, False, deck)
                break
            # rank playable cards for defender
    if (len(cards_played) < upper_limit or (len(cards_played) == upper_limit and cards_played[-1][1] == "")) and pc != 0:
        if cards_played[-1][1] == "":
            back_propogate(False, nodes, True, deck)
            # defender wins
        else:
            back_propogate(True, nodes, True, deck)
            # attacker wins
    return high, low, multiple, pass_c


def update_json(defending):
    file_name = ""
    nodes = []
    if defending:
        file_name = "Defender_simple.json"
    else:
        file_name = "Attacker_simple.json"
    for node in Node.nodes:
        node = Node.nodes[node]
        node_dict = {}
        node_dict["Name"] = node.name
        node_dict["Strategy"] = node.strategy
        node_dict["Player"] = node.player
        node_dict["Parents"] = node.parent
        node_dict["Children"] = node.children
        node_dict["Wins"] = node.wins
        node_dict["Simulations"] = node.simulations
        nodes.append(node_dict)
    with open(file_name, 'w') as file:
        json.dump(nodes, file)

def reset_JSON():
    def_tree = [{"Name": 0,"Strategy": "Start","Player": "Attacker","Parents": [],"Children": [1,2],"Wins": 2,"Simulations": 2},{"Name": 1,"Strategy": "Highest","Player": "Defender","Parents": 0,"Children": [],"Wins": 1,"Simulations":1},{"Name": 2,"Strategy": "Lowest","Player": "Defender","Parents": 0,"Children": [],"Wins": 1,"Simulations":1}]
    att_tree = [{"Name": 0, "Strategy": "Start", "Player": "Defender", "Parents": [], "Children": [1,2], "Wins": 2, "Simulations": 2},{"Name": 1,"Strategy": "Highest","Player": "Attacker","Parents": 0,"Children": [],"Wins": 1,"Simulations":1},{"Name": 2,"Strategy": "Lowest","Player": "Attacker","Parents": 0,"Children": [],"Wins": 1,"Simulations":1}]
    with open("Defender_simple.json", 'w') as file:
        json.dump(def_tree, file)
    with open("Attacker_simple.json", 'w') as file:
        json.dump(att_tree, file)

def choose_action(h,l,m,p):
    vals = [h,l,m,p]
    max_val = max(vals)
    out = []
    for v in vals:
        if v == max_val:
            out.append(vals.index(v))
    if len(out) == 1:
        return out[0]
    choice = random.choice(out)
    return choice

## possibly add deck into the fold, lower card get more win whn deck is more full etc
## also may need to limit the tree growth to 12 levels
def runsimulations(hand, trump, cards_played, defending, testhand_len = 6, deck = 54):
    tree = ""
    if not defending:
        parse_file('Attacker_simple.json')
        tree = "Attacker"
    else:
        parse_file('Defender_simple.json')
        tree = "Defender"

    attacker = topological()
    # print(bn)
    # print(Node.nodes[bn[0]].simulations)
    # print(UCB1_Score(Node.nodes[bn[1]]))
    DurakEnv.find_playable(cards_played, hand, defending, trump)
    pc = DurakEnv.count_playable(hand)
    start = Node.nodes[0]
    high = 0
    low = 0
    multiple = 0
    pass_c = 0
    testhand = generate_hand(hand, trump, testhand_len)
    DurakEnv.find_playable(cards_played, testhand, not defending, trump)
    mult_hand = DurakEnv.mult_possible(DurakEnv.get_amounts_per_number(hand))
    mult_testhand = DurakEnv.mult_possible(DurakEnv.get_amounts_per_number(testhand))
    for i in range(10):
        test = Node.nodes
        Strategy = parse_tree(start, mult_hand, mult_testhand, tree)
        test = Strategy[-1]
        if len(Strategy) < 12 and Strategy[-1].strategy != "Pass":
            expandnode(Strategy[-1])
        high_one, low_one, m_one, p_one = Simulate(hand.copy(), testhand.copy(), defending, trump, cards_played.copy(), Strategy, deck)
        high += high_one
        low += low_one
        multiple += m_one
        pass_c += p_one
    update_json(defending)
    DurakEnv.find_playable(cards_played, hand, defending, trump)
    pc = DurakEnv.count_playable(hand)
    if not mult_hand:
        h, l = DurakEnv.get_hand_ranks([hand], trump, cards_played)
    else:
        h,l,m = DurakEnv.get_hand_ranks([hand], trump, cards_played, mult = True)
    choice = choose_action(high,low,multiple,pass_c)
    if choice == 2 and len(m) == 0:
        choice = choose_action(high, low, -1, pass_c)
    if choice == 0:
        return h
    elif choice == 1:
        return l
    elif choice == 2:
        pos_sel = []
        ranks = []
        for k in m:
            pos_sel.append(choose_multiple(m[k], trump, hand))
            ranks.append(hand[pos_sel[-1]].rank())
            if hand[pos_sel[-1]].suit() == trump.suit():
                ranks[-1] += 13
        if high > low:
            return pos_sel[ranks.index(max(ranks))]
        elif low > high:
            return pos_sel[ranks.index(min(ranks))]
        else:
            if len(pos_sel) == 1:
                return pos_sel[0]
            return random.choice(pos_sel)
    else:
        return "p"