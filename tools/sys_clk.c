#include <stdio.h>

int main() {

   for (int target = 50; target <= 400; target += 25) {
      // VCO needs to be in range 400MHz to 1600MHz
      // So with a 12MHz clock, the
      int crystal = 12;
      for (int refdiv = 1; refdiv < 63; refdiv++) {
         int min_mult =  400 * refdiv / 12;
         int max_mult = 1600 * refdiv / 12;
         if (min_mult < 16) {
            min_mult = 16;
         }
         if (max_mult > 320) {
            max_mult = 320;
         }
         for (int mult = min_mult; mult <= max_mult; mult++) {
            for (int pd1 = 1; pd1 <= 7; pd1++) {
               for (int pd2 = 1; pd2 <= 7; pd2++) {
                  if (12 * mult == target * pd1 * pd2) {
                     printf("%4d = (12 / %2d) * %4d / %1d / %1d\n", target, refdiv, mult, pd1, pd2);
                  }
               }
            }
         }
      }
   }
}
