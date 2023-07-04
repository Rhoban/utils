#include <algorithm>
#include <vector>

namespace rhoban_utils
{
template <typename T>
class LexicographicalScore
{
public:
  LexicographicalScore() : score()
  {
  }

  LexicographicalScore(T value) : score({ value })
  {
  }

  void operator=(T value)
  {
    score = { value };
  }

  T& operator[](size_t index)
  {
    if (score.size() < index + 1)
    {
      score.resize(index + 1);
    }

    return score[index];
  }

  bool operator<(const LexicographicalScore<T>& other) const
  {
    return std::lexicographical_compare(score.begin(), score.end(), other.score.begin(), other.score.end());
  }

  operator T() const
  {
    return score[0];
  }

  friend std::ostream& operator<<(std::ostream& os, const LexicographicalScore<T>& obj)
  {
    for (size_t i = 0; i < obj.score.size(); i++)
    {
      os << obj.score[i];
      if (i < obj.score.size() - 1)
        os << ", ";
    }

    return os;
  }

  std::vector<T> score;
};
}  // namespace rhoban_utils