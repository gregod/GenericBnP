use std::marker::PhantomData;
use std::slice::Iter;
use std::time::Instant;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ColumnId(pub u32);

impl std::hash::Hash for ColumnId {
    fn hash<H: std::hash::Hasher>(&self, hasher: &mut H) {
        hasher.write_u32(self.0)
    }
}

impl nohash_hasher::IsEnabled for ColumnId {}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
/// Column ticket is used to ensure that no columns
/// are generated twice within a multi-threaded environment
/// Each worker records the time it started generated columns.
/// When submitting new columns it uses the ticket to identify
/// columns added by other workers in the meantime.
pub struct ColumnTicket(pub usize);

#[derive(Clone, Debug, PartialEq)]
pub struct Column<ColumnType> {
    pub id: ColumnId,
    pub data: ColumnType,
}

/// Holds all columns generated so far
pub struct ColumnPool<ColumnType, FilterType> {
    local_column_counter: u32,
    columns: Vec<Column<ColumnType>>,
    phantom: PhantomData<FilterType>,
}

impl<ColumnType, FilterType> ColumnPool<ColumnType, FilterType> {
    pub fn new() -> Self {
        ColumnPool {
            local_column_counter: 0,
            columns: Vec::new(),
            phantom: PhantomData,
        }
    }
}

/// Trait to be implemented by the user.
/// Given a list of branching filters, return only those
/// columns that remain valid.
pub trait ColumnPoolFilter<ColumnType, FilterType> {
    fn get_columns(&self, filters: &[FilterType], ticket : Option<ColumnTicket>) -> (ColumnTicket, Vec<&Column<ColumnType>>);
}

impl<ColumnType, FilterType> ColumnPool<ColumnType, FilterType> {

    /// Total number of columns in pool
    pub fn count(&self) -> usize {
        self.columns.len()
    }

    /// Returns a specific column from the pool
    pub fn get_column(&self, id: ColumnId) -> &Column<ColumnType> {
        let column_at_index = &self.columns[id.0 as usize];

        // make sure that our assumption that columns are ordered in the column pool is correct
        // i.e. ids of lookup key and value match
        debug_assert_eq!(column_at_index.id.0, id.0);

        column_at_index
    }

    /// Get Iterator of all columns in column pool and associated ticket
    pub fn get_all_columns(&self) -> (ColumnTicket,Iter<Column<ColumnType>>) {
        (ColumnTicket(self.columns.len()), self.columns.iter())
    }


    /// Adds a column to the column pool
    /// The `column_ticket` that was handed out during get_columns 
    /// must be returned, to ensure column pool consistency in a 
    /// multithreaded environment
    pub fn add_column(&mut self, column_data: ColumnType, column_ticket: ColumnTicket) -> bool
    where
        ColumnType: PartialEq,
    {



        // in an multi threaded environment, another thread
        // might have added columns inbetween
        // we got a ticket="num_cols known" when retrieving the columns
        // we then only need to check those columns > ticket_no for potential conflicts
        // if there is a conflict, return the previously added variable instead

        let existing_column = self
            .columns
            .iter()
            .enumerate() // uses enumerate and indecies due to lifetime constraints
            .skip(column_ticket.0)
            .filter_map(|( i ,c)| if c.data == column_data { Some(i)} else { None})
            .next();


        if let Some(column) = existing_column {
            // there was the same column within the pool
            return false;

        } else {


            #[cfg(feature = "validity_assertions")]
            {
                // assert that we never regenerate the same column twice
                let existing_columns  = self
                    .columns
                    .iter()
                    .filter(|c| c.data == column_data)
                    .next();

                    assert!(existing_columns.is_none());
            }


            let column = Column {
                id: ColumnId(self.local_column_counter),
                data: column_data,
            };

            self.local_column_counter += 1;
            self.columns.push(column);

            return true

        }
    }
}

impl<ColumnType, FilterType> Clone for ColumnPool<ColumnType, FilterType>
where
    ColumnType: Clone,
{
    fn clone(&self) -> Self {
        Self {
            local_column_counter: self.local_column_counter,
            columns: self.columns.clone(),
            phantom: PhantomData,
        }
    }
}
