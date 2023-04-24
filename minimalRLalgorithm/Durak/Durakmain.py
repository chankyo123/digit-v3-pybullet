from cards import Deck, Card
import MonteCarlo
import gym
from gym import logger, spaces
from gym.envs.classic_control import utils
from gym.error import DependencyNotInstalled

class DurakEnv(gym.Env):
    def __init__(self):
        self.FIRST_ATTACK = True
        self.state = None
        self.steps_beyond_terminated = None


    def find_upper_limit(defender_hand, attacker_hand, mc):
        if not mc:
            return min(len(defender_hand), 6)
        else:
            return min([len(defender_hand), len(attacker_hand), 6])


    def update_turn_order(self,turns, taken):
        if not taken:
            if len(turns[0]) == 1:
                turns[1], turns[0][0] = turns[0][0], turns[1]
            else:
                defender = turns[1]
                next_defender = defender + 1
                while next_defender not in turns[0]:
                    next_defender = next_defender + 1
                    if next_defender > max(turns[0]):
                        next_defender = 0
                if next_defender in turns[0]:
                    turns[0].pop(turns[0].index(next_defender))
                turns[1] = next_defender
                turns[0].append(defender)
                turns[0].sort()
                while turns[0][0] != defender:
                    turns[0].append(turns[0].pop(0))
        else:
            if len(turns[0]) != 1:
                defender = turns[1]
                attacker = defender + 1
                while not (attacker in turns[0] or attacker == turns[1]):
                    attacker = attacker + 1
                    if attacker > max(turns[0]):
                        attacker = 0
                next_defender = attacker + 1
                while next_defender not in turns[0]:
                    next_defender = next_defender + 1
                    if next_defender == attacker:
                        next_defender += 1
                    if next_defender > max(turns[0]):
                        next_defender = 0

                turns[1] = next_defender
                if next_defender in turns[0]:
                    turns[0].pop(turns[0].index(next_defender))
                turns[0].append(defender)
                turns[0].sort()
                while turns[0][0] != attacker:
                    turns[0].append(turns[0].pop(0))


    def reorder(self,players, trump):
        cozar = trump.suit()
        for player in players:
            s1 = []
            s2 = []
            s3 = []
            s4 = []
            order = [s1, s2, s3, s4]
            for card in player:
                if card.suit() == 1:
                    s1.append(card)
                    if cozar == 1 and order[0] == s1:
                        order.pop(0)
                        order.append(s1)
                elif card.suit() == 2:
                    s2.append(card)
                    if cozar == 2 and order[1] == s2:
                        order.pop(1)
                        order.append(s2)
                elif card.suit() == 3:
                    s3.append(card)
                    if cozar == 3 and order[2] == s3:
                        order.pop(2)
                        order.append(s3)
                else:
                    s4.append(card)
                    if cozar == 4 and order[3] == s4:
                        order.pop(3)
                        order.append(s4)
            s1.sort()
            s2.sort()
            s3.sort()
            s4.sort()
            player.clear()
            for s in order:
                player.extend(s)


    def find_low_trump(self,player_hands, trump):
        cozar = trump.suit()
        player_num = 0
        low_rank = 15
        for hand in player_hands:
            for card in hand:
                if card.suit() == cozar:
                    if low_rank > card.rank():
                        low_rank = card.rank()
                        player_num = player_hands.index(hand)
        return player_num


    def display_cards_played(self,cards):
        out = ""
        temp = ""
        for group in cards:
            if group[1] != "":
                temp = str(group[0]) + ", " + str(group[1])
            else:
                temp = str(group[0])
            out += temp
            out += " "
        out = out[0:-1]
        return out


    def display(self,players, trump, defending, cards_played, deck_size):
        out = ""
        count = len(players[1])
        if len(players) == 4:
            count = max([len(players[1]), len(players[3])])
        for i in range(count + 2):
            if i == 0:
                out += "\t"
                if len(players) >= 3:
                    for card in players[2]:
                        out += str(card)
                        out += " "
                    if len(players[2]) < 6:
                        curr = len(players[2])
                        while curr < 6:
                            out += "  "
                            curr += 1
                else:
                    out += "\t\t\t\t\t\t\t\t"
            elif i == count + 1:
                out += "\t"
                for card in players[0]:
                    out += str(card)
                    out += " "
            else:
                if i <= len(players[1]):
                    out += str(players[1][i - 1])
                    out += " "
                else:
                    out += "  "
                if defending == 1 and cards_played and i <= len(cards_played):
                    pair = cards_played[i - 1]
                    if pair[1] == "":
                        temp = str(pair[0]) + "   "
                    else:
                        temp = str(pair[0]) + ", " + str(pair[1])
                    out += temp
                    out += "\t\t"
                else:
                    out += "\t\t\t\t"
                if i == 1 and defending == 2 and cards_played:
                    out += self.display_cards_played(cards_played)
                elif i == (count + 2) // 2:
                    out += str(trump)
                    out += " [" + str(deck_size) + "]\t\t"
                elif i == count and defending == 0 and cards_played:
                    out += self.display_cards_played(cards_played)
                else:
                    out += " "
                    if defending == 3 and cards_played and i <= len(cards_played):
                        pair = cards_played[i - 1]
                        if pair[1] == "":
                            temp = str(pair[0]) + "   "
                        else:
                            temp = str(pair[0]) + ", " + str(pair[1])
                        out += temp
                        out += "\t\t"
                    else:
                        out += "\t\t\t\t"
                if len(players) == 4:
                    if i <= len(players[3]):
                        out += str(players[3][i - 1])
                    else:
                        out += "  "
            out += "\n"
        print(out)


    def get_amounts_per_number(self,hand):
        hand_dict = {}
        for card in hand:
            if card.rank() not in hand_dict:
                hand_dict[card.rank()] = 1
            else:
                hand_dict[card.rank()] += 1
        return hand_dict


    def mult_possible(self,hand_dict):
        for i, v in hand_dict.items():
            if v > 1:
                return True
        return False


    def random_card_choice(self,hand, cards_played, defender, trump, test_hand_len=6, deck=54):
        pc = self.count_playable(hand)
        if pc != 1:
            return MonteCarlo.runsimulations(hand.copy(), trump, cards_played.copy(), defender, test_hand_len, deck)
        else:
            for card in hand:
                if card.is_playable():
                    return hand.index(card)


    def reset_playable(self,hand):
        for card in hand:
            if card.is_playable():
                card.change_playable()


    def count_playable(self,hand):
        i = 0
        for card in hand:
            if card.is_playable():
                i += 1
        return i


    def take_cards(self,hand, cards_played, human, play_num):
        for pair in cards_played:
            for card in pair:
                if card != '':
                    if not human:
                        if not card.is_face_up():
                            card.flip_card()
                    else:
                        if play_num == 0 and not card.is_face_up():
                            card.flip_card()
                    hand.append(card)
        cards_played.clear()


    def find_playable(self,cards_played, hand, defending, cozar):
        if defending:
            for pair in cards_played:
                if pair[1] == "":
                    for card in hand:
                        cr = card.rank()
                        if cr == 1:
                            cr = 14
                        pr = pair[0].rank()
                        if pr == 1:
                            pr = 14
                        if (card.suit() == pair[0].suit() and cr > pr) or (
                                card.suit() == cozar.suit() and pair[0].suit() != cozar.suit()):
                            if not card.is_playable():
                                card.change_playable()
        else:
            if cards_played:
                nums_played = []
                for pair in cards_played:
                    if pair[0].rank() not in nums_played:
                        nums_played.append(pair[0].rank())
                    if pair[1] != "" and pair[1] not in nums_played:
                        nums_played.append(pair[1].rank())
                for card in hand:
                    if card.rank() in nums_played:
                        if not card.is_playable():
                            card.change_playable()
            else:
                for card in hand:
                    if not card.is_playable():
                        card.change_playable()


    def deal_cards(self,turns, deck, players, trump, human):
        for turn in turns[0]:
            while len(players[turn]) < 6:
                if not deck.is_empty():
                    players[turn].append(deck.deal())
                elif not trump.is_used():
                    players[turn].append(trump)
                    trump.use()
                else:
                    break
        while len(players[turns[1]]) < 6:
            if not deck.is_empty():
                players[turns[1]].append(deck.deal())
            elif not trump.is_used():
                players[turns[1]].append(trump)
                trump.use()
            else:
                break
        if human.lower() == 'y':
            for player in players:
                if players.index(player) != 0:
                    for card in player:
                        if card.is_face_up():
                            card.flip_card()


    def check_for_durak(self,deck, trump, players, turns):
        finished = False
        durak = None
        if not deck.is_empty() or not trump.is_used():
            finished = False
        else:
            still_playing = 0
            playing = []
            for player in players:
                if len(player) != 0:
                    still_playing += 1
                    playing.append(players.index(player))
            if still_playing > 1:
                finished = False
            elif still_playing == 1:
                finished = True
                durak = playing[0]
            else:
                finished = True
                durak = turns[1]
        return finished, durak


    def filter_playable(self,hand_values, hand):
        final_vals = []
        for card in hand:
            if card.is_playable():
                id = hand.index(card)
                final_vals.append((hand_values[0][id], id))
        return final_vals


    def gather_mult_vals(self,hand_vals, hand):
        finals = {}
        for card in hand:
            if card.is_playable():
                id = hand.index(card)
                if card.rank() not in finals:
                    finals[card.rank()] = [[id], []]
                else:
                    finals[card.rank()][0].append(id)
        for card in hand:
            if not card.is_playable():
                id = hand.index(card)
                if card.rank() in finals:
                    finals[card.rank()][1].append(id)
        deletion = []
        for k, v in finals.items():
            if len(v[0]) + len(v[1]) < 2:
                deletion.append(k)
        for k in deletion:
            del finals[k]
        return finals


    def get_hand_ranks(self,players, trump, cards_played, trump_constant=14, mult=False, passing=False):
        cozar = trump.suit()
        prev_ranked_cards = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0, 9: 0, 10: 0, 11: 0, 12: 0, 13: 0, 14: 0}
        hand_ranks = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0, 9: 0, 10: 0, 11: 0, 12: 0, 13: 0, 14: 0}
        prev_suited_cards = {1: 0, 2: 0, 3: 0, 4: 0}
        hand_values = []

        for prev_cards in cards_played:
            prev_ranked_cards[prev_cards[0].rank()] += 1
            prev_suited_cards[prev_cards[0].suit()] += 1

        for player_hand in players:
            hand_value = []

            for card in player_hand:
                hand_ranks[card.rank()] += 1

            for card in player_hand:
                card_value = card.rank_val()
                if card.suit() == cozar:
                    card_value += trump_constant
                else:
                    card_value -= prev_suited_cards[card.suit()]
                card_value += hand_ranks[card.rank()]
                card_value -= prev_ranked_cards[card.rank()]
                hand_value.append(card_value)

            hand_values.append(hand_value)

        if len(players) == 1:
            if passing:
                return self.filter_playable(hand_values, players[0])
            else:
                final = self.filter_playable(hand_values, players[0])
                min_id = min(final)[1]
                max_id = max(final)[1]
                if mult:
                    mult_ids = self.gather_mult_vals(hand_values[0], players[0])
                    return (min_id, max_id, mult_ids)
                return (min_id, max_id)
        return hand_values


    def construct_value_state(self,cards_played, defending, trump, hand_values):
        state = []
        state.append(len(cards_played))
        state.append(defending)
        state.append(trump.suit())
        state.extend(hand_values[0])
        return state


    def construct_omniscient_state(self,cards_played, defending, trump, hand_values):
        state = []
        state.append()
        pass


    def remove_players(self, players, turns, trump):
        if trump.is_used():
            for player in players:
                if len(player) == 0:
                    turn = players.index(player)
                    if turn not in turns[0] and turn != turns[1]:
                        continue
                    if turns[1] == turn:
                        if len(turns[0]) != 1:
                            next = turn + 1
                            while next not in turns[0]:
                                next += 1
                                if next > len(players):
                                    next = 0
                            turns[1] = next
                            turns[0].pop(turns[0].index(next))
                    else:
                        if len(turns[0]) != 1:
                            turns[0].pop(turns[0].index(turn))

    def step(self, action):
        deck, trump, players, turns, turn, cards_played, attacking = self.state[0],self.state[1],self.state[2],self.state[3],self.state[4],self.state[5],,self.state[6]
        if trump.is_used():
            finished, durak = self.check_for_durak(deck, trump, players, turns)
            if not finished:
                if len(players[0]) == 0:
                    reward = 1.0
            else:
                if durak == 0:
                    reward = -1.0
        else:
            if attacking:
                
    # Press the green button in the gutter to run the script.
    def run_durak_env(self):
        #    MonteCarlo.reset_JSON()
        should_continue = True
        first = True
        while should_continue:
            deck = Deck()
            deck.shuffle()
            trump = deck.deal()

            players = []
            cards_played = []
            num_players = int(input("Enter number of players (2-4): "))
            while num_players not in [2, 3, 4]:
                print("Number not valid. Try again.")
                num_players = int(input("Enter number of players (2-4): "))
            human = input("Do you want to join? (y or n): ")
            while human.lower() not in ["y", "n"]:
                print("Answer not understood. Try again.")
                human = input("Do you want to join? (y or n): ")
            for player in range(num_players):
                players.append([])

            for i in range(6):
                for player in players:
                    player.append(deck.deal())
            if human.lower() == "y":
                for i in range(1, len(players)):
                    for card in players[i]:
                        card.flip_card()
            self.reorder(players, trump)
            if first:
                initial_attacker = self.find_low_trump(players, trump)
            else:
                if durak > len(players):
                    initial_attacker = self.find_low_trump(players, trump)
                else:
                    initial_attacker = durak + 1
                    if initial_attacker > (len(players) - 1):
                        initial_attacker = 0
            initial_attacker = 0
            first = False
            upper_limit = 6
            defender = initial_attacker + 1
            if defender > num_players - 1:
                defender = 0
            temp = []
            turns = [temp]
            for i in range(num_players):
                if i == defender:
                    turns.append(defender)
                else:
                    temp.append(i)
            while temp[0] != initial_attacker:
                temp.append(temp.pop(0))
            turn = 0
            attacking = True
            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
            self.display(players, trump, defender, cards_played, len(deck))

            print()
            i = 0
            finished = False
            durak = None
            while not finished:
                finished, durak = self.check_for_durak(deck, trump, players, turns)
                if finished:
                    continue
                hand_values = self.get_hand_ranks(players, trump, cards_played)
                state = self.construct_value_state(cards_played, defender, trump, hand_values)
                if attacking:
                    if human.lower() == "y" and turns[0][turn] == 0:
                        self.find_playable(cards_played, players[0], False, trump)
                        card = input("Choose a card to play (Enter number 1-{} or p to pass): ".format(len(players[0])))
                    if card.lower() != "p":
                        temp = int(card)
                    else:
                        temp = card
                    possible = list(range(1, len(players[0]) + 1))
                    possible.append('p')
                    possible.append('P')
                    while temp not in possible or (type(temp) == int and not players[0][int(card) - 1].is_playable()):
                        print("Number not valid. Try again.")
                        card = input("Choose a card to play (Enter number 1-{} or p to pass): ".format(len(players[0])))
                        if card.lower() != "p":
                            temp = int(card)
                        else:
                            temp = card
                        possible = list(range(1, len(players[0]) + 1))
                        possible.append('p')
                        possible.append('P')
                        if card.lower() == "p":
                            turn += 1
                            if turn == len(turns[0]):
                                turn = 0
                            self.state = [deck, trump, players, turns, turn, cards_played, attacking]

                        else:
                            card = int(card)
                            if players[0][card - 1].is_playable():
                                players[0][card - 1].change_playable()
                            cards_played.append((players[0][card - 1], ""))
                            players[0].pop(card - 1)
                            self.state = [deck, trump, players, turns, turn, cards_played, attacking]
                        self.FIRST_ATTACK = False
                    else:
                        self.find_playable(cards_played, players[turns[0][turn]], False, trump)
                        pc = self.count_playable(players[turns[0][turn]])
                        testing = 0
                        while pc == 0 and len(cards_played) < 6 and testing < len(turns[0]):
                            turn += 1
                            if turn == len(turns[0]):
                                turn = 0
                            self.find_playable(cards_played, players[turns[0][turn]], False, trump)
                            pc = self.count_playable(players[turns[0][turn]])
                            testing += 1
                        if testing == len(turns[0]):
                            self.reset_playable(players[turns[0][turn]])
                            cards_played.clear()
                            self.deal_cards(turns, deck, players, trump, human)
                            self.remove_players(players, turns, trump)
                            if trump.is_used():
                                finished, durak = self.check_for_durak(deck, trump, players, turns)
                                self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                                if finished:
                                    continue
                            self.reorder(players, trump)
                            self.update_turn_order(turns, False)
                            defender = turns[1]
                            self.FIRST_ATTACK = True
                            attacking = True
                            turn = 0
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]

                        self.find_playable(cards_played, players[turns[0][turn]], False, trump)
                        choice = self.random_card_choice(players[turns[0][turn]], cards_played, False, trump,
                                                    len(players[defender]), len(deck))
                        self.FIRST_ATTACK = False
                        if choice == "p":
                            turn += 1
                            if turn == len(turns[0]):
                                turn = 0
                        else:
                            if not players[turns[0][turn]][choice].is_face_up():
                                players[turns[0][turn]][choice].flip_card()
                            players[turns[0][turn]][choice].change_playable()
                            cards_played.append((players[turns[0][turn]][choice], ""))
                            players[turns[0][turn]].pop(choice)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                    attacking = False
                    self.reset_playable(players[turns[0][turn]])
                    self.display(players, trump, defender, cards_played, len(deck))
                    self.find_playable(cards_played, players[defender], True, trump)
                    self.display(players, trump, defender, cards_played, len(deck))
                else:
                    if human.lower() == "y" and defender == 0:
                        self.find_playable(cards_played, players[defender], True, trump)
                        pc = self.count_playable(players[defender])
                        if pc == 0:
                            self.take_cards(players[defender], cards_played, True, 0)
                            self.deal_cards(turns, deck, players, trump, human)
                            self.remove_players(players, turns, trump)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                            if trump.is_used():
                                finished, durak = self.check_for_durak(deck, trump, players, turns)
                                self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                                if finished:
                                    continue
                            self.reorder(players, trump)
                            self.update_turn_order(turns, True)
                            defender = turns[1]
                            self.FIRST_ATTACK = True
                            turn = 0
                            upper_limit = self.find_upper_limit(players[defender], [], False)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                            continue
                        card = input("Choose a card to play (Enter number 1-{} or p to pass): ".format(len(players[0])))
                        if card.lower() != "p":
                            temp = int(card)
                        else:
                            temp = card
                        possible = list(range(1, len(players[0]) + 1))
                        possible.append('p')
                        possible.append('P')
                        while temp not in possible or (type(temp) == int and not players[0][int(card) - 1].is_playable()):
                            print("Number not valid. Try again.")
                            card = input("Choose a card to play (Enter number 1-{} or p to pass): ".format(len(players[0])))
                            if card.lower() != "p":
                                temp = int(card)
                            else:
                                temp = card
                            possible = list(range(1, len(players[0]) + 1))
                            possible.append('p')
                            possible.append('P')
                        if card.lower() == "p":
                            self.take_cards(players[defender], cards_played, True, defender)
                            self.deal_cards(turns, deck, players, trump, human)
                            self.remove_players(players, turns, trump)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                            if trump.is_used():
                                finished, durak = self.check_for_durak(deck, trump, players, turns)
                                self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                                if finished:
                                    continue
                            self.reorder(players, trump)
                            self.update_turn_order(turns, True)
                            defender = turns[1]
                            self.FIRST_ATTACK = True
                            attacking = True
                            turn = 0
                            self.display(players, trump, defender, cards_played, len(deck))
                            upper_limit = self.find_upper_limit(players[defender], [], False)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                            continue
                        else:
                            card = int(card)
                            out = cards_played[-1]
                            if not players[0][card - 1].is_face_up():
                                players[0][card - 1].flip_card()
                            players[defender][card - 1].change_playable()
                            cards_played[-1] = (out[0], players[0][card - 1])
                            players[0].pop(card - 1)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                    else:
                        self.find_playable(cards_played, players[defender], True, trump)
                        pc = self.count_playable(players[defender])
                        if pc == 0:
                            self.take_cards(players[defender], cards_played, False, defender)
                            self.deal_cards(turns, deck, players, trump, human)
                            self.remove_players(players, turns, trump)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                            if trump.is_used():
                                finished, durak = self.check_for_durak(deck, trump, players, turns)
                                self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                                if finished:
                                    continue
                            self.reorder(players, trump)
                            self.update_turn_order(turns, True)
                            defender = turns[1]
                            self.FIRST_ATTACK = True
                            attacking = True
                            turn = 0
                            self.display(players, trump, defender, cards_played, len(deck))
                            upper_limit = self.find_upper_limit(players[defender], [], False)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                            continue
                        choice = self.random_card_choice(players[defender], cards_played, True, trump,
                                                    len(players[turns[0][turn]]), len(deck))
                        self.FIRST_ATTACK = False
                        if choice == "p":
                            self.take_cards(players[defender], cards_played, False, defender)
                            self.deal_cards(turns, deck, players, trump, human)
                            self.remove_players(players, turns, trump)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                            if trump.is_used():
                                finished, durak = self.check_for_durak(deck, trump, players, turns)
                                self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                                if finished:
                                    continue
                            self.reorder(players, trump)
                            self.update_turn_order(turns, True)
                            defender = turns[1]
                            self.FIRST_ATTACK = True
                            attacking = True
                            turn = 0
                            self.display(players, trump, defender, cards_played, len(deck))
                            upper_limit = self.find_upper_limit(players[defender], [], False)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                            continue
                        else:
                            out = cards_played[-1]
                            if not players[defender][choice].is_face_up():
                                players[defender][choice].flip_card()
                            players[defender][choice].change_playable()
                            cards_played[-1] = (out[0], players[defender][choice])
                            players[defender].pop(choice)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                    self.display(players, trump, defender, cards_played, len(deck))
                    self.reset_playable(players[defender])
                    self.find_playable(cards_played, players[turns[0][turn]], False, trump)
                    pc = self.count_playable(players[turns[0][turn]])
                    testing = 0
                    while pc == 0 and len(cards_played) < upper_limit and testing < len(turns[0]):
                        turn += 1
                        if turn == len(turns[0]):
                            turn = 0
                        self.find_playable(cards_played, players[turns[0][turn]], False, trump)
                        pc = self.count_playable(players[turns[0][turn]])
                        testing += 1
                    if testing == len(turns[0]) or len(cards_played) == upper_limit:
                        cards_played.clear()
                        self.deal_cards(turns, deck, players, trump, human)
                        self.remove_players(players, turns, trump)
                        self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                        if trump.is_used():
                            finished, durak = self.check_for_durak(deck, trump, players, turns)
                            self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                            if finished:
                                continue
                        self.reorder(players, trump)
                        self.update_turn_order(turns, False)
                        defender = turns[1]
                        self.FIRST_ATTACK = True
                        attacking = True
                        upper_limit = self.find_upper_limit(players[defender], [], False)
                        turn = 0
                        self.state = [deck, trump, players, turns,turn, cards_played, attacking]
                    self.display(players, trump, defender, cards_played, len(deck))
                    attacking = True
            print("Game finished! Player ", durak, " was the durak!")
            should_c = input("Do you want to play again? (y or n)")
            if should_c.lower() != "y":
                should_continue = False