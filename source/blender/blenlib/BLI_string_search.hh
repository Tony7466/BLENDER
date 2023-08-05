/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_linear_allocator.hh"
#include "BLI_span.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

struct StringSearch;

StringSearch *BLI_string_search_new();
/**
 * Add a new possible result to the search.
 * The caller keeps ownership of all parameters.
 *
 * \param weight: Can be used to customize the order when multiple items have the same match score.
 */
void BLI_string_search_add(StringSearch *search, const char *str, void *user_data, int weight);

/**
 * Filter and sort all previously added search items.
 * Returns an array containing the filtered user data.
 * The caller has to free the returned array.
 */
int BLI_string_search_query(StringSearch *search, const char *query, void ***r_data);
void BLI_string_search_free(StringSearch *search);

namespace blender::string_search {

template<typename T> class StringSearchNew {
 private:
  StringSearch *string_search_;

  static_assert(std::is_pointer_v<T>);

 public:
  StringSearchNew()
  {
    string_search_ = BLI_string_search_new();
  }

  ~StringSearchNew()
  {
    BLI_string_search_free(string_search_);
  }

  void add(const StringRefNull str, T user_data, const int weight = 0)
  {
    BLI_string_search_add(string_search_, str.c_str(), (void *)user_data, weight);
  }

  Vector<T> query(const StringRefNull query) const
  {
    T *results;
    const int amount = BLI_string_search_query(string_search_, query.c_str(), (void ***)&results);
    Vector<T> results_vec = Span(results, amount);
    MEM_freeN(results);
    return results_vec;
  }
};

/**
 * Computes the cost of transforming string a into b. The cost/distance is the minimal number of
 * operations that need to be executed. Valid operations are deletion, insertion, substitution and
 * transposition.
 *
 * This function is utf8 aware in the sense that it works at the level of individual code points
 * (1-4 bytes long) instead of on individual bytes.
 */
int damerau_levenshtein_distance(StringRef a, StringRef b);
/**
 * Returns -1 when this is no reasonably good match.
 * Otherwise returns the number of errors in the match.
 */
int get_fuzzy_match_errors(StringRef query, StringRef full);
/**
 * Splits a string into words and normalizes them (currently that just means converting to lower
 * case). The returned strings are allocated in the given allocator.
 */
void extract_normalized_words(StringRef str,
                              LinearAllocator<> &allocator,
                              Vector<StringRef, 64> &r_words);

}  // namespace blender::string_search
