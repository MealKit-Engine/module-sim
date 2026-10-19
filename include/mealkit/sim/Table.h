#pragma once

#include "mealkit/core/Reflect.h"

#include <string>
#include <string_view>
#include <vector>

// 정의 데이터 표 (docs/CODING_MODEL.md "같은 모양의 데이터가 여러 개"): 무기·아이템·적 종류처럼 같은 모양의 행이 여러 개인 데이터.
//   MEALKIT_STRUCT(PropKind) { MEALKIT_FIELD(f32, density, 200.0f, "밀도"); ... };
//   MEALKIT_TABLE(PropKind, "tables/props.table", 0);   // 시뮬레이션 표: 콘텐츠 해시에 들어간다
// 파일(assets/tables/*.table)은 "row <id>" 줄 다음에 그 행의 "경로 값" 줄들. 에디터 [표] 패널에서 고치고 CSV로 주고받는다.
// 매치를 시작할 때 app/이 data::loadTable로 읽어 World에 복사한다. 다른 데이터는 행을 RowRef<Row>(이름 id)로 가리키고,
// 불러온 뒤 data::resolveRows가 행 번호를 채운다. sim/은 번호로 읽는다 (행 순서 = 파일 순서, 모든 피어가 같다).
namespace mealkit::sim {

template <class Row>
class Table {
public:
    using RowType = Row;

    u32 size() const { return static_cast<u32>(rows.size()); }
    const Row& operator[](u32 index) const { return rows[index]; }
    Row& operator[](u32 index) { return rows[index]; }
    const std::string& id(u32 index) const { return ids[index]; }
    // 행 참조 → 행 (참조가 비었거나 없는 id면 nullptr)
    const Row* get(const RowRef<Row>& ref) const { return ref.index < rows.size() ? &rows[ref.index] : nullptr; }
    // 이름으로 찾기 (불러올 때·화면 쪽용. 시뮬레이션 틱 안에서는 번호를 쓴다). 없으면 kNoRow
    u32 find(std::string_view rowId) const {
        for (u32 i = 0; i < ids.size(); ++i)
            if (ids[i] == rowId) return i;
        return kNoRow;
    }

    std::vector<std::string> ids; // 행 이름 (파일 순서)
    std::vector<Row> rows;
};

} // namespace mealkit::sim
