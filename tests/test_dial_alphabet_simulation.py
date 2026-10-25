import unittest
import numpy as np
from scripts.simulate_dial_alphabets import all_codes,evaluate,coordinate_distribution
from scripts.simulate_dial_feedback import entropy_tree,coordinate_record


class AlphabetSimulationTests(unittest.TestCase):
    def test_base_encoding_enumerates_each_password_once(self):
        codes=all_codes(2,4)
        self.assertEqual(codes.shape,(16,2))
        self.assertEqual(codes[0].tolist(),[1,1]);self.assertEqual(codes[-1].tolist(),[4,4])
        self.assertEqual(len(np.unique(codes,axis=0)),16)

    def test_three_symbols_reproduce_previous_policy(self):
        for n in (2,3):
            codes=all_codes(n,3);old=entropy_tree(codes,'record',incumbent_neighbors=True)
            result,ids,queries,moves=evaluate(n,3)
            self.assertTrue(result['exhaustive'])
            np.testing.assert_array_equal(queries,old['queries'])
            np.testing.assert_array_equal(moves,old['moves'])

    def test_coordinate_polynomial_matches_full_three_symbol_enumeration(self):
        for n in (1,2,4):
            actual=coordinate_record(all_codes(n,3))['queries'];expected=coordinate_distribution(n,3)
            self.assertAlmostEqual(expected['mean'],float(np.mean(actual)))
            for p in (5,20,50,80,95):
                self.assertEqual(expected['percentiles'][f'P{p}'],int(np.quantile(actual,p/100,method='inverted_cdf')))

    def test_one_dial_coordinate_distribution_is_uniform_one_to_k(self):
        for k in (4,5,6,7):
            d=coordinate_distribution(1,k)
            self.assertEqual(d['mean'],(k+1)/2)
            self.assertEqual(d['guaranteed_maximum'],k)


if __name__=='__main__':unittest.main()
