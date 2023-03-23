package com.carming.backend.place.repository;

import com.carming.backend.place.domain.Place;
import com.carming.backend.place.domain.PlaceCategory;
import com.carming.backend.place.dto.request.PlaceSearch;
import com.querydsl.core.types.dsl.BooleanExpression;
import com.querydsl.jpa.impl.JPAQueryFactory;
import lombok.RequiredArgsConstructor;

import java.util.List;

import static com.carming.backend.place.domain.QPlace.place;

@RequiredArgsConstructor
public class PlaceRepositoryImpl implements PlaceRepositoryCustom {

    private final JPAQueryFactory queryFactory;

    @Override
    public List<Place> getPlaces(PlaceSearch search) {
        return queryFactory
                .selectFrom(place)
                .where(regionEq(search.getRegions()), categoryEq(search.getCategory()))
                .orderBy(place.ratingSum.desc())
                .fetch();
    }

    private BooleanExpression regionEq(List<String> regions) {
        return place.region.in(regions);
    }

    private BooleanExpression categoryEq(PlaceCategory category) {
        return category != null ? place.category.eq(category) : null;
    }
}
