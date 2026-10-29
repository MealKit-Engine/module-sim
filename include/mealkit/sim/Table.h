#pragma once

#include "mealkit/core/Reflect.h"

#include <string>
#include <string_view>
#include <vector>

// Definition data tables (docs/CODING_MODEL.md "Many items of the same shape"): data with many rows of the same shape, like weapons/items/enemy kinds.
//   MEALKIT_STRUCT(PropKind) { MEALKIT_FIELD(f32, density, 200.0f, "Density"); ... };
//   MEALKIT_TABLE(PropKind, "tables/props.table", 0);   // simulation table: goes into the content hash
// The file (assets/tables/*.table) has a "row <id>" line followed by that row's "path value" lines. Edited in the editor [Tables] panel and exchanged as CSV.
// When a match starts, app/ reads it with data::loadTable and copies it into the World. Other data points to rows with RowRef<Row> (name id),
// and data::resolveRows fills in row indices after loading. sim/ reads by index (row order = file order, the same on all peers).
namespace mealkit::sim {

template <class Row>
class Table {
public:
    using RowType = Row;

    u32 size() const { return static_cast<u32>(rows.size()); }
    const Row& operator[](u32 index) const { return rows[index]; }
    Row& operator[](u32 index) { return rows[index]; }
    const std::string& id(u32 index) const { return ids[index]; }
    // Row reference → row (nullptr if the reference is empty or the id is missing)
    const Row* get(const RowRef<Row>& ref) const { return ref.index < rows.size() ? &rows[ref.index] : nullptr; }
    // Find by name (for loading/view side. Use indices inside simulation ticks). kNoRow if missing
    u32 find(std::string_view rowId) const {
        for (u32 i = 0; i < ids.size(); ++i)
            if (ids[i] == rowId) return i;
        return kNoRow;
    }

    std::vector<std::string> ids; // row names (file order)
    std::vector<Row> rows;
};

} // namespace mealkit::sim
